#!/usr/bin/env sh
set -eu

CFLAGS="-std=c11 -Wall -Wextra -DSTM32F103xB"
INCLUDES="-ICore/Inc -ICore/Inc/User_code -IDrivers/STM32F1xx_HAL_Driver/Inc -IDrivers/CMSIS/Device/ST/STM32F1xx/Include -IDrivers/CMSIS/Include"

gcc $CFLAGS $INCLUDES \
  test_dxl_protocol2.c Core/Src/User_code/dxl/dxl_protocol2.c \
  -o /tmp/test_dxl_protocol2
/tmp/test_dxl_protocol2

gcc $CFLAGS $INCLUDES \
  test_xl330_hal_host.c Core/Src/User_code/dxl/dxl_protocol2.c Core/Src/User_code/dxl/dxl_xl330_driver.c \
  -o /tmp/test_xl330_hal_host
/tmp/test_xl330_hal_host

# Giao thức PC <-> STM32. HAL và driver XL330 thay bằng bản giả trong file test.
# CMSIS cảnh báo cast con trỏ khi biên dịch cho host 64 bit, không liên quan code ta.
gcc $CFLAGS -Wno-int-to-pointer-cast $INCLUDES \
  test_pc_link_host.c Core/Src/User_code/pc_link.c \
  -o /tmp/test_pc_link_host
/tmp/test_pc_link_host
