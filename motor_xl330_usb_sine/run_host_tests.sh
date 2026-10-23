#!/usr/bin/env sh
# pc_link + motion chạy trên PC, HAL/USB/XL330 là bản giả trong test_host.c.
# CMSIS cảnh báo cast con trỏ khi biên dịch cho host 64 bit, không liên quan code ta.
set -eu
gcc -std=gnu11 -Wall -Wextra -Werror -Wno-int-to-pointer-cast -DSTM32F103xB \
  -ICore/Inc -IDrivers/STM32F1xx_HAL_Driver/Inc -IDrivers/CMSIS/Device/ST/STM32F1xx/Include -IDrivers/CMSIS/Include \
  test_host.c Core/Src/User_code/pc_link.c Core/Src/User_code/motion.c -lm -o /tmp/test_usb_sine_host
/tmp/test_usb_sine_host
