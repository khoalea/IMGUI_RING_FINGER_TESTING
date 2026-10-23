# XL330 sine + USB CDC, STM32F103C8T6.  make / make clean / make flash
TARGET    = motor_xl330_usb_sine
BUILD_DIR = build

C_SOURCES = \
Core/Src/main.c \
Core/Src/stm32f1xx_it.c \
Core/Src/stm32f1xx_hal_msp.c \
Core/Src/system_stm32f1xx.c \
Core/Src/sysmem.c \
Core/Src/syscalls.c \
Core/Src/User_code/pc_link.c \
Core/Src/User_code/motion.c \
$(wildcard Core/Src/User_code/dxl/*.c) \
$(wildcard Drivers/STM32F1xx_HAL_Driver/Src/*.c) \
USB_DEVICE/App/usb_device.c \
USB_DEVICE/App/usbd_desc.c \
USB_DEVICE/App/usbd_cdc_if.c \
USB_DEVICE/Target/usbd_conf.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
ASM_SOURCES = Core/Startup/startup_stm32f103c8tx.s

C_INCLUDES = \
-ICore/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
-IDrivers/CMSIS/Include \
-IUSB_DEVICE/App \
-IUSB_DEVICE/Target \
-IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
-IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc

PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
SZ = $(PREFIX)size
CP = $(PREFIX)objcopy

MCU    = -mcpu=cortex-m3 -mthumb
CFLAGS = $(MCU) -DUSE_HAL_DRIVER -DSTM32F103xB $(C_INCLUDES) -Og -g -std=gnu11 \
         -Wall -fdata-sections -ffunction-sections -MMD -MP
# Code viết cho bản này phải sạch -Wextra -Werror; HAL/USB của ST và thư viện dxl/ có sẵn thì không ép.
OWN_CFLAGS = -Wextra -Werror
LDFLAGS = $(MCU) -specs=nano.specs -TSTM32F103C8TX_FLASH.ld -lc -lm -lnosys \
          -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--gc-sections -Wl,--print-memory-usage

OBJECTS = $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o) $(ASM_SOURCES:.s=.o))

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/Core/Src/main.o $(BUILD_DIR)/Core/Src/User_code/pc_link.o $(BUILD_DIR)/Core/Src/User_code/motion.o: CFLAGS += $(OWN_CFLAGS)

$(BUILD_DIR)/%.o: %.c Makefile
	@mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile
	@mkdir -p $(dir $@)
	$(CC) -x assembler-with-cpp -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(CP) -O binary -S $< $@

flash: $(BUILD_DIR)/$(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program $< verify reset exit"

clean:
	rm -rf $(BUILD_DIR)

-include $(OBJECTS:.o=.d)
.PHONY: all clean flash
