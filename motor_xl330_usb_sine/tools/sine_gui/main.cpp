// sine_gui: operator console cho firmware motor_xl330_usb_sine qua USB CDC.
// GUI = main thread (core riêng), I/O serial = thread khác. Hai bên chỉ nói qua queue không khóa.
// Vòng điều khiển nằm trên MCU, nên thread I/O không cần SCHED_FIFO.
#include "proto.hpp"
#include "rt.hpp"
#include "imgui.h"
#include "imgui_internal.h"   // DockBuilder*
#include "implot.h"
#ifdef GUI_HEADLESS
#include "imgui_impl_null.h"
#else
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#endif
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

static double now_s() { timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return double(ts.tv_sec) + double(ts.tv_nsec) * 1e-9; }

// ---------------------------------------------------------------- messages (POD) -------------
struct Sample   { std::uint32_t t_ms; pcl::TlmRec r; };                       // io -> GUI, mỗi motor mỗi khung TLM
struct Reply    { std::uint8_t type, len; std::uint8_t pay[pcl::kMaxPayload]; }; // io -> GUI, mọi khung khác TLM
struct Tx       { std::uint8_t len; std::uint8_t buf[pcl::kMoveLen + 6]; };   // GUI -> io, khung đã đóng gói
struct LinkStat { bool open; int err; std::uint32_t bad; std::uint64_t rx_frames, tlm_frames; };

struct Links {
  rt::SpscQueue<Sample> tlm{1 << 14};
  rt::SpscQueue<Reply>  rep{64};
  rt::SpscQueue<Tx>     tx{64};
  rt::Latest<LinkStat>  stat;
  std::atomic<bool>     run{true};
};

static Tx frame(std::uint8_t type, const std::uint8_t* pay, std::uint8_t len) {
  Tx t{}; t.len = static_cast<std::uint8_t>(pcl::build(t.buf, type, pay, len)); return t;
}

// ---------------------------------------------------------------- I/O thread -----------------
static int open_port(const char* path, int& err) {
  const int fd = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) { err = errno; return -1; }
  termios tio{};
  if (tcgetattr(fd, &tio) != 0) { err = errno; ::close(fd); return -1; }
  cfmakeraw(&tio);                                  // baud vô nghĩa với USB CDC
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cc[VMIN] = 0; tio.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &tio) != 0) { err = errno; ::close(fd); return -1; }
  ioctl(fd, TIOCEXCL);                              // tiến trình khác (vd. haptic_hand_node) không mở chen vào được
  tcflush(fd, TCIOFLUSH);
  err = 0;
  return fd;
}

static bool write_all(int fd, const Tx& t) {
  std::size_t done = 0;
  for (int tries = 0; done < t.len && tries < 50; ++tries) {   // ponytail: thử tối đa 50 x 1 ms rồi coi là đứt
    const ssize_t n = ::write(fd, t.buf + done, t.len - done);
    if (n > 0) { done += std::size_t(n); continue; }
    if (n < 0 && errno != EAGAIN && errno != EINTR) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return done == t.len;
}

static void io_thread(Links& L, const char* port, int core) {
  rt::name_this_thread("io");
  rt::pin_this_thread(core);
  int fd = -1; double t_try = -1;
  pcl::Decoder d; LinkStat st{};
  std::uint8_t buf[512];
  auto drop = [&](int e) { ::close(fd); fd = -1; st.err = e; };

  while (L.run.load(std::memory_order_relaxed)) {
    if (fd < 0) {
      st.open = false; L.stat.write(st);
      if (now_s() - t_try < 0.5) { std::this_thread::sleep_for(std::chrono::milliseconds(50)); continue; }
      t_try = now_s();
      if ((fd = open_port(port, st.err)) < 0) continue;
      Tx stale; while (L.tx.try_pop(stale)) {}       // lệnh xếp hàng lúc đứt link là lệnh cũ: bỏ, không gửi bù
      d = pcl::Decoder{};
      const std::uint8_t rate[2] = {10, 0};          // TLM 10 ms = tick motion của firmware
      if (!write_all(fd, frame(pcl::CMD_PING, nullptr, 0)) || !write_all(fd, frame(pcl::CMD_TLM_RATE, rate, 2))) {
        drop(EIO); continue;
      }
    }
    Tx t;
    while (fd >= 0 && L.tx.try_pop(t)) if (!write_all(fd, t)) drop(EIO);
    if (fd < 0) continue;

    pollfd p{fd, POLLIN, 0};
    if (::poll(&p, 1, 20) < 0 && errno != EINTR) { drop(errno); continue; }
    if (p.revents & (POLLHUP | POLLERR | POLLNVAL)) { drop(ENODEV); continue; }   // rút cáp
    const ssize_t n = (p.revents & POLLIN) ? ::read(fd, buf, sizeof buf) : 0;
    if (n < 0 && errno != EAGAIN && errno != EINTR) { drop(errno); continue; }

    for (ssize_t i = 0; i < n; ++i) {
      if (!d.feed(buf[i])) continue;
      ++st.rx_frames;
      if (d.type == pcl::RSP_TLM) {
        ++st.tlm_frames;                            // link sống kể cả khi n = 0 motor
        const std::uint8_t cnt = d.len >= 5 ? d.pay[4] : 0;
        if (d.len != 5 + pcl::kTlmRec * cnt) { ++d.bad; continue; }
        const auto t_ms = pcl::rd<std::uint32_t>(d.pay);
        for (std::uint8_t k = 0; k < cnt; ++k) L.tlm.try_push({t_ms, pcl::unpack_tlm(d.pay + 5 + pcl::kTlmRec * k)});
      } else {
        Reply r{d.type, d.len, {}}; std::memcpy(r.pay, d.pay, d.len); L.rep.try_push(r);
      }
    }
    st.open = true; st.bad = d.bad; L.stat.write(st);
  }
  if (fd >= 0) {                                    // GUI đóng: tắt torque mọi motor đã được GUI gửi xuống tx
    Tx t; while (L.tx.try_pop(t)) (void)write_all(fd, t);
    ::close(fd);
  }
}

// ---------------------------------------------------------------- GUI-side state -------------
struct Scrolling {              // ring cố định, ImPlot đọc qua Offset: không cấp phát mỗi frame
  std::vector<float> x, y; int cap, n = 0, off = 0;
  explicit Scrolling(int c) : x(std::size_t(c)), y(std::size_t(c)), cap(c) {}
  void push(float px, float py) {
    if (n < cap) { x[std::size_t(n)] = px; y[std::size_t(n)] = py; ++n; }
    else { x[std::size_t(off)] = px; y[std::size_t(off)] = py; off = (off + 1) % cap; }
  }
};

// XL330: 4096 pulse/vòng (0.088 °/pulse); Profile Velocity 0.229 rpm/LSB -> 0.229*4096/60 pulse/s/LSB.
static constexpr float kDegPerPulse = 360.0f / 4096.0f, kPulsePerSPerVelLsb = 0.229f * 4096.0f / 60.0f;

struct Motor {
  std::uint8_t id;
  pcl::Move want{};             // thứ người dùng đang chỉnh
  pcl::Move echo{}; bool has_echo = false;   // bản firmware đã kẹp, nguồn sự thật
  bool center_init = false; float freq_hz = 0.5f; std::int32_t pos = 0;
  // Số chu kỳ = tích phân f·dt theo t_ms của MCU, đúng cách firmware tăng pha. Về 0 khi Start (firmware reset pha).
  double cycles = 0; std::uint32_t last_t_ms = 0; bool have_t = false;
  Scrolling goal{4096}, meas{4096}, cur{4096};
  explicit Motor(std::uint8_t i) : id(i) { want.id = i; want.amp = 300; want.freq_mhz = 500; want.goal_cur = 0xFFFF; }
  bool running() const { return has_echo && echo.enable; }
};

struct Ui {
  Links& L; const char* port;
  std::vector<Motor> motors;    // ponytail: tối đa vài motor, tìm tuyến tính là đủ
  LinkStat ls{}; double last_tlm = -1; std::uint64_t seen_tlm = 0; float t_latest = 0, window_s = 10;
  char last_err[96] = ""; long bus_baud = -1;   // -1 = chưa có PONG, 0 = firmware không thấy motor
  std::uint64_t tx_dropped = 0;

  Motor& motor(std::uint8_t id) {
    for (auto& m : motors) if (m.id == id) return m;
    motors.emplace_back(id); return motors.back();
  }
  void send(const Tx& t) { if (!L.tx.try_push(t)) ++tx_dropped; }
  void send_move(const pcl::Move& m) { std::uint8_t p[pcl::kMoveLen]; pcl::pack_move(p, m); send(frame(pcl::CMD_MOVE, p, pcl::kMoveLen)); }
  void stop_all() {
    for (auto& m : motors) { pcl::Move s = m.has_echo ? m.echo : m.want; s.id = m.id; s.enable = 0; send_move(s); }
  }
  bool stale() const { return last_tlm < 0 || now_s() - last_tlm > 0.25; }

  void ingest() {
    Sample s; int budget = 4096;
    while (budget-- && L.tlm.try_pop(s)) {
      Motor& m = motor(s.r.id);
      const float t = float(s.t_ms) * 1e-3f; t_latest = t;
      m.pos = s.r.pos;
      if (m.have_t && m.running()) m.cycles += double(m.echo.freq_mhz) * 1e-3 * double(s.t_ms - m.last_t_ms) * 1e-3;
      m.last_t_ms = s.t_ms; m.have_t = true;
      if (!m.center_init) { m.want.center = s.r.pos; m.center_init = true; }   // bật torque không giật: center = chỗ đang đứng
      m.goal.push(t, float(s.r.goal)); m.meas.push(t, float(s.r.pos)); m.cur.push(t, float(s.r.cur));
    }
    Reply r;
    while (L.rep.try_pop(r)) {
      if (r.type == pcl::RSP_PONG && r.len >= 2) {
        for (int i = 0; i < r.pay[1] && 2 + i < r.len; ++i) motor(r.pay[2 + i]);
        if (r.len >= 6 + r.pay[1]) bus_baud = long(pcl::rd<std::uint32_t>(r.pay + 2 + r.pay[1]));
      }
      else if (r.type == pcl::RSP_MOVE && r.len == pcl::kMoveLen) { auto e = pcl::unpack_move(r.pay); Motor& m = motor(e.id);
        if (e.enable && !m.running()) m.cycles = 0;           // bật từ OFF: firmware chạy lại từ pha 0
        m.echo = e; m.has_echo = true; }
      else if (r.type == pcl::RSP_ERR && r.len == 2) {
        static const char* names[] = {"?", "BAD_LEN", "UNKNOWN_CMD", "BUS", "?", "PARAM"};
        std::snprintf(last_err, sizeof last_err, "ERR %s cho lệnh 0x%02X", names[r.pay[0] < 6 ? r.pay[0] : 0], r.pay[1]);
      }
    }
    L.stat.read(ls);
    if (ls.tlm_frames != seen_tlm) { seen_tlm = ls.tlm_frames; last_tlm = now_s(); }
  }
};

// ---------------------------------------------------------------- panels ---------------------
static void panel_control(Ui& ui) {
  if (!ImGui::Begin("Control")) { ImGui::End(); return; }
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.10f, 0.10f, 1));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.90f, 0.15f, 0.15f, 1));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(1.00f, 0.25f, 0.25f, 1));
  if (ImGui::Button("STOP ALL - torque off  (Space)", ImVec2(-FLT_MIN, ImGui::GetFrameHeight() * 2.2f)) ||
      (ImGui::IsKeyPressed(ImGuiKey_Space, false) && !ImGui::GetIO().WantTextInput)) ui.stop_all();
  ImGui::PopStyleColor(3);
  if (ImGui::Button("Ping (quét lại ID)")) ui.send(frame(pcl::CMD_PING, nullptr, 0));
  if (ui.motors.empty()) ImGui::TextWrapped("Chưa thấy motor. Firmware dò baud + ID một lần lúc khởi động (~5 s): cấp nguồn motor rồi cắm lại USB.");

  const bool stale = ui.stale();
  for (std::size_t i = 0; i < ui.motors.size(); ++i) {
    Motor& m = ui.motors[i];
    ImGui::PushID(m.id);
    char hdr[48]; std::snprintf(hdr, sizeof hdr, "Motor ID %u  %s###hdr", m.id, m.running() ? "[CHẠY]" : "[torque off]");
    if (ImGui::CollapsingHeader(hdr, ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::InputInt("center [pulse]", &m.want.center);
      ImGui::SameLine(); if (ImGui::SmallButton("= pos")) m.want.center = m.pos;
      // Ô nhập số: gõ thẳng giá trị, nút -/+ bước nhỏ, Ctrl + -/+ bước lớn.
      if (ImGui::InputInt("amp [pulse]", &m.want.amp, 10, 100)) m.want.amp = std::clamp(m.want.amp, 0, 4095);
      ImGui::SameLine(); ImGui::TextDisabled("±%.1f°", float(m.want.amp) * kDegPerPulse);
      if (ImGui::InputFloat("freq [Hz]", &m.freq_hz, 0.1f, 1.0f, "%.3f")) {
        m.freq_hz = std::clamp(m.freq_hz, 0.0f, 10.0f);          // trần firmware MOTION_MAX_FREQ_MHZ
        m.want.freq_mhz = std::uint32_t(std::lround(m.freq_hz * 1000.0f));
      }
      int vel = int(m.want.prof_vel), cur = m.want.goal_cur;
      if (ImGui::InputInt("vel [0.229 rpm]", &vel)) m.want.prof_vel = std::uint32_t(vel < 0 ? 0 : vel);
      ImGui::SetItemTooltip("Profile Velocity raw. 0 = Velocity Limit của motor");
      if (ImGui::InputInt("torque [raw]", &cur)) m.want.goal_cur = std::uint16_t(cur < 0 ? 0 : cur > 0xFFFF ? 0xFFFF : cur);
      ImGui::SetItemTooltip("Goal Current raw, firmware kẹp về Current Limit (addr 38)");

      ImGui::BeginDisabled(stale);                  // không gửi lệnh khi không thấy telemetry sống
      if (!m.running()) {
        if (ImGui::Button("Start...")) { ImGui::OpenPopup("Xác nhận bật torque"); }
      } else {
        if (ImGui::Button("Update")) { m.want.enable = 1; ui.send_move(m.want); }
        ImGui::SameLine(); if (ImGui::Button("Stop")) { pcl::Move s = m.want; s.enable = 0; ui.send_move(s); }
      }
      ImGui::EndDisabled();
      if (ImGui::BeginPopupModal("Xác nhận bật torque", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Motor ID %u sẽ có torque và bắt đầu chạy sin.", m.id);
        ImGui::Text("center %d, pos hiện tại %d (lệch %d pulse)", m.want.center, m.pos, m.want.center - m.pos);
        if (ImGui::Button("Bật", ImVec2(120, 0))) { m.want.enable = 1; ui.send_move(m.want); ImGui::CloseCurrentPopup(); }
        ImGui::SameLine(); if (ImGui::Button("Huỷ", ImVec2(120, 0))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
      }

      if (m.has_echo) {
        const auto& e = m.echo;
        if (m.running() || m.cycles > 0)
          ImGui::Text("Chu kỳ hoàn thành: %lld  (+%.0f%%)%s", (long long)m.cycles, (m.cycles - std::floor(m.cycles)) * 100.0,
                      m.running() ? "" : "  [đã dừng]");
        ImGui::TextDisabled("firmware đã kẹp: amp %d  f %.3f Hz  vel %u  torque %u", e.amp, e.freq_mhz * 1e-3, e.prof_vel, e.goal_cur);
        const float need = 2.0f * 3.14159265f * float(e.freq_mhz) * 1e-3f * float(e.amp);   // tốc độ đỉnh sin, pulse/s
        if (e.prof_vel && float(e.prof_vel) * kPulsePerSPerVelLsb < need)
          ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.2f, 1), "vel thấp hơn tốc độ đỉnh sin (cần >= %.0f): sin sẽ bẹt",
                             std::ceil(need / kPulsePerSPerVelLsb));
      }
    }
    ImGui::PopID();
  }
  ImGui::End();
}

static void panel_status(Ui& ui) {
  if (!ImGui::Begin("Status")) { ImGui::End(); return; }
  const ImVec4 ok(0.3f, 0.85f, 0.4f, 1), bad(0.95f, 0.3f, 0.3f, 1);
  if (!ui.ls.open) ImGui::TextColored(bad, "%s: không mở được (%s)", ui.port, std::strerror(ui.ls.err));
  else ImGui::TextColored(ui.stale() ? bad : ok, "%s: %s", ui.port, ui.stale() ? "MỞ, KHÔNG CÓ TLM" : "LINK OK");
  if (ui.ls.err == EBUSY) ImGui::TextWrapped("Cổng đang bị tiến trình khác giữ (haptic_hand_node?). `fuser %s`", ui.port);
  ImGui::Text("rx frames %llu, khung hỏng %u", (unsigned long long)ui.ls.rx_frames, ui.ls.bad);
  ImGui::Text("drop: tlm %llu, rep %llu, tx %llu", (unsigned long long)ui.L.tlm.dropped(),
              (unsigned long long)ui.L.rep.dropped(), (unsigned long long)(ui.L.tx.dropped() + ui.tx_dropped));
  if (ui.bus_baud > 0) ImGui::Text("bus Dynamixel: %ld baud", ui.bus_baud);
  else if (ui.bus_baud == 0) ImGui::TextColored(bad, "bus Dynamixel: không baud nào có motor trả lời");
  if (ui.last_err[0]) ImGui::TextColored(bad, "%s", ui.last_err);
  ImGui::Text("gui %.0f FPS, core %d", ImGui::GetIO().Framerate, rt::current_core());
  ImGui::End();
}

static void line(const char* id, const Scrolling& s) {
  ImPlot::PlotLine(id, s.x.data(), s.y.data(), s.n, ImPlotSpec(ImPlotProp_Offset, s.off));
}

static void panel_plots(Ui& ui) {
  if (!ImGui::Begin("Plots")) { ImGui::End(); return; }
  ImGui::SetNextItemWidth(160); ImGui::SliderFloat("window [s]", &ui.window_s, 1, 40, "%.0f");
  const double t = ui.t_latest;
  if (ImPlot::BeginSubplots("##sub", 2, 1, ImVec2(-1, -1), ImPlotSubplotFlags_LinkAllX, nullptr)) {
    if (ImPlot::BeginPlot("Position")) {
      ImPlot::SetupAxes("t MCU [s]", "[pulse]", 0, ImPlotAxisFlags_AutoFit);
      ImPlot::SetupAxisLimits(ImAxis_X1, t - ui.window_s, t, ImPlotCond_Always);
      char id[24];
      for (auto& m : ui.motors) {
        std::snprintf(id, sizeof id, "goal %u", m.id); line(id, m.goal);
        std::snprintf(id, sizeof id, "pos %u", m.id);  line(id, m.meas);
      }
      ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("Present Current")) {
      ImPlot::SetupAxes("t MCU [s]", "[raw]", 0, ImPlotAxisFlags_AutoFit);
      ImPlot::SetupAxisLimits(ImAxis_X1, t - ui.window_s, t, ImPlotCond_Always);
      char id[24];
      for (auto& m : ui.motors) { std::snprintf(id, sizeof id, "cur %u", m.id); line(id, m.cur); }
      ImPlot::EndPlot();
    }
    ImPlot::EndSubplots();
  }
  ImGui::End();
}

static void draw_ui(Ui& ui) {
  const ImGuiID dock = ImGui::DockSpaceOverViewport();
  static bool built = false;                        // layout mặc định chỉ khi imgui.ini chưa có
  if (!built) { built = true;
    if (ImGui::DockBuilderGetNode(dock) == nullptr || ImGui::DockBuilderGetNode(dock)->IsLeafNode()) {
      ImGui::DockBuilderRemoveNode(dock);
      ImGui::DockBuilderAddNode(dock, ImGuiDockNodeFlags_DockSpace);
      ImGui::DockBuilderSetNodeSize(dock, ImGui::GetMainViewport()->Size);
      ImGuiID left, right, left_bottom;
      ImGui::DockBuilderSplitNode(dock, ImGuiDir_Left, 0.32f, &left, &right);
      ImGui::DockBuilderSplitNode(left, ImGuiDir_Down, 0.22f, &left_bottom, &left);
      ImGui::DockBuilderDockWindow("Control", left);
      ImGui::DockBuilderDockWindow("Status", left_bottom);
      ImGui::DockBuilderDockWindow("Plots", right);
      ImGui::DockBuilderFinish(dock);
    }
  }
  panel_control(ui); panel_status(ui); panel_plots(ui);
}

// ---------------------------------------------------------------- main = GUI thread ----------
int main(int argc, char** argv) {
  const int ncpu = int(std::thread::hardware_concurrency());
  int gui_core = ncpu - 1, io_core = ncpu > 1 ? ncpu - 2 : 0, frames = -1;
  const char* port = "/dev/ttyACM0";
  for (int i = 1; i + 1 < argc; i += 2) {
    if (!std::strcmp(argv[i], "--port"))     port = argv[i + 1];
    if (!std::strcmp(argv[i], "--gui-core")) gui_core = std::atoi(argv[i + 1]);
    if (!std::strcmp(argv[i], "--io-core"))  io_core = std::atoi(argv[i + 1]);
    if (!std::strcmp(argv[i], "--frames"))   frames = std::atoi(argv[i + 1]);
  }
  // Pin TRƯỚC glfwInit: thread của driver GPU thừa kế mask này, ở yên trên GUI core.
  rt::name_this_thread("gui");
  if (!rt::pin_this_thread(gui_core)) std::fprintf(stderr, "warn: cannot pin GUI to core %d\n", gui_core);

  Links L;
  std::thread io(io_thread, std::ref(L), port, io_core);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext(); ImPlot::CreateContext();
  ImGuiIO& io_cfg = ImGui::GetIO();
  io_cfg.ConfigFlags |= ImGuiConfigFlags_DockingEnable | ImGuiConfigFlags_NavEnableKeyboard;
  ImGui::StyleColorsDark();
  // Font mặc định chỉ có ASCII; ImGui 1.92 nạp glyph theo nhu cầu nên không cần khai báo dải tiếng Việt.
  if (!io_cfg.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 15.0f))
    std::fprintf(stderr, "warn: thiếu DejaVuSans.ttf, chữ có dấu sẽ hiện ?\n");

#ifdef GUI_HEADLESS
  ImGui_ImplNullPlatform_Init(); ImGui_ImplNullRender_Init();
  if (frames < 0) frames = 120;
#else
  if (!glfwInit()) return 1;
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  const float scale = ImGui_ImplGlfw_GetContentScaleForMonitor(glfwGetPrimaryMonitor());
  GLFWwindow* win = glfwCreateWindow(int(1400 * scale), int(850 * scale), "XL330 sine console", nullptr, nullptr);
  if (!win) return 1;
  glfwMakeContextCurrent(win);
  glfwSwapInterval(1);
  ImGui::GetStyle().ScaleAllSizes(scale); ImGui::GetStyle().FontScaleDpi = scale;
  ImGui_ImplGlfw_InitForOpenGL(win, true); ImGui_ImplOpenGL3_Init("#version 330");
#endif

  Ui ui{L, port, {}};
  for (int f = 0; frames < 0 || f < frames; ++f) {
#ifdef GUI_HEADLESS
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
    ImGui_ImplNullPlatform_NewFrame(); ImGui_ImplNullRender_NewFrame();
#else
    glfwPollEvents();
    if (glfwWindowShouldClose(win)) break;
    if (glfwGetWindowAttrib(win, GLFW_ICONIFIED)) { ui.ingest(); ImGui_ImplGlfw_Sleep(50); continue; }
    ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame();
#endif
    ui.ingest();
    ImGui::NewFrame();
    draw_ui(ui);
    ImGui::Render();
#ifndef GUI_HEADLESS
    int w, h; glfwGetFramebufferSize(win, &w, &h);
    glViewport(0, 0, w, h); glClearColor(0.08f, 0.08f, 0.09f, 1); glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(win);
#endif
  }
#ifdef GUI_HEADLESS
  std::printf("headless: frames=%d port=%s open=%d err=%s rx=%llu motors=%zu tlm_dropped=%llu gui_core=%d\n",
              frames, port, ui.ls.open, std::strerror(ui.ls.err), (unsigned long long)ui.ls.rx_frames,
              ui.motors.size(), (unsigned long long)L.tlm.dropped(), rt::current_core());
#endif
  ui.stop_all();                                    // firmware không có deadman: tự tắt torque trước khi thoát
  L.run = false; io.join();
#ifdef GUI_HEADLESS
  ImGui_ImplNullRender_Shutdown(); ImGui_ImplNullPlatform_Shutdown();
#else
  ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown();
#endif
  ImPlot::DestroyContext(); ImGui::DestroyContext();
#ifndef GUI_HEADLESS
  glfwDestroyWindow(win); glfwTerminate();
#endif
  return 0;
}
