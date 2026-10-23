/**
 * @file    motion.h
 * @brief   Bảng lệnh chuyển động cho từng motor: vị trí chạy theo hình sin.
 *
 *     goal = center + amp * sin(phase),   phase tiến 2*pi*f mỗi giây
 *
 * pc_link.c ghi vào bảng khi nhận lệnh MOVE, main.c đọc bảng mỗi tick và
 * đẩy xuống bus. Cả hai chạy trong vòng lặp chính, không có ISR nào đụng tới.
 *
 * Motor chạy ở Current-based Position (mode 5): Goal Position bám hình sin,
 * Profile Velocity là trần tốc độ, Goal Current là trần mô-men.
 */

#ifndef INC_MOTION_H_
#define INC_MOTION_H_

#include <stdint.h>
#include "User_code/dxl/dxl_xl330_driver.h"

/* Tần số sin tối đa, mHz. Ở tick 10ms, 10Hz chỉ còn 10 điểm mỗi chu kỳ. */
#define MOTION_MAX_FREQ_MHZ     10000u

typedef struct {
    /* ---- PC đặt qua lệnh MOVE ---- */
    uint8_t  enable;      /* 0 = tắt torque, motor thả lỏng                 */
    int32_t  center;      /* pulse                                          */
    int32_t  amp;         /* pulse, 0 = đứng giữ ở center                   */
    uint32_t freq_mhz;    /* mHz                                            */
    uint32_t prof_vel;    /* Profile Velocity (addr 112), raw 0.229 rpm/LSB.
                           * 0 = KHÔNG giới hạn tốc độ                      */
    uint16_t goal_cur;    /* Goal Current (addr 102), raw. Driver cũ coi
                           * 1 LSB = 1 mA, chưa đối chiếu e-Manual XL330    */
    /* ---- firmware tự giữ ---- */
    uint8_t  dirty;       /* 1 = enable/prof_vel/goal_cur vừa đổi, phải ghi */
    float    phase;       /* rad, [0, 2pi)                                  */
    int32_t  goal;        /* goal position vừa gửi                          */
    uint16_t cur_lim;     /* Current Limit (addr 38) đọc lúc init, để kẹp   */
    uint32_t vel_lim;     /* Velocity Limit (addr 44) đọc lúc init, để kẹp  */
} MotionCh;

extern MotionCh motion[XL330_MAX_MOTORS];   /* chỉ số = thứ tự trong dxl */

/* Tiến pha thêm dt_ms rồi trả về goal position (pulse). Hàm thuần, không I/O. */
int32_t Motion_Step(MotionCh *c, uint32_t dt_ms);

#endif /* INC_MOTION_H_ */
