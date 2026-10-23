cmake_minimum_required(VERSION 3.20)
project(sine_gui CXX C)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
option(GUI_HEADLESS "ImGui null backend (CI/SSH, không mở cửa sổ)" OFF)

enable_testing()
add_executable(test_proto test_proto.cpp)                 # không phụ thuộc ImGui
target_compile_options(test_proto PRIVATE -Wall -Wextra -Werror)
add_test(NAME test_proto COMMAND test_proto)

include(FetchContent)
# Ghim tag: ImGui/ImPlot đổi API giữa các bản. Có sẵn bản local thì trỏ
# -DFETCHCONTENT_SOURCE_DIR_IMGUI=... -DFETCHCONTENT_SOURCE_DIR_IMPLOT=... để khỏi tải.
FetchContent_Declare(imgui  GIT_REPOSITORY https://github.com/ocornut/imgui  GIT_TAG v1.92.9b-docking GIT_SHALLOW ON)
FetchContent_Declare(implot GIT_REPOSITORY https://github.com/epezent/implot GIT_TAG v1.0             GIT_SHALLOW ON)
FetchContent_MakeAvailable(imgui implot)

add_library(imgui_all STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp ${imgui_SOURCE_DIR}/imgui_draw.cpp ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${implot_SOURCE_DIR}/implot.cpp ${implot_SOURCE_DIR}/implot_items.cpp)
target_include_directories(imgui_all SYSTEM PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends ${implot_SOURCE_DIR})

if(GUI_HEADLESS)
  target_sources(imgui_all PRIVATE ${imgui_SOURCE_DIR}/backends/imgui_impl_null.cpp)
  target_compile_definitions(imgui_all PUBLIC GUI_HEADLESS)
else()
  find_package(glfw3 3.3 QUIET)
  if(NOT glfw3_FOUND)
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(glfw GIT_REPOSITORY https://github.com/glfw/glfw GIT_TAG 3.4 GIT_SHALLOW ON)
    FetchContent_MakeAvailable(glfw)
  endif()
  find_package(OpenGL REQUIRED)
  target_sources(imgui_all PRIVATE ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
                                   ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
  target_link_libraries(imgui_all PUBLIC glfw OpenGL::GL)
endif()

find_package(Threads REQUIRED)
add_executable(sine_gui main.cpp)
target_link_libraries(sine_gui PRIVATE imgui_all Threads::Threads)
target_compile_options(sine_gui PRIVATE -Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion)
