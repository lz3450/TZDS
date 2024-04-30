project := $(notdir $(shell pwd))

################################################################
# build config
################################################################
build = debug
# build = release

################################################################
# cmse config
################################################################
# cmse = s
cmse = ns

################################################################
# project configuration
################################################################
includedirs =
asm =
src =
sdk_config =

################################
# common
################################
includedirs += CMSIS/Core/Include
includedirs += LPC55S69
includedirs += drivers
includedirs += components
includedirs += utilities

## mutually exclusive options
## core 0
asm += startup/startup_LPC55S69_cm33_core0.S
## core 1
# asm += startup/startup_LPC55S69_cm33_core1.S

src += LPC55S69/system_LPC55S69_cm33_core0.c

src += drivers/fsl_common.c
src += drivers/fsl_common_arm.c
src += drivers/fsl_clock.c
src += drivers/fsl_power.c
src += drivers/fsl_reset.c

## mutually exclusive options
## core 0
sdk_config += CPU_LPC55S69JBD100_cm33_core0
## core 1
# sdk_config += CPU_LPC55S69JBD100_cm33_core1

################################
# drivers/fsl_gpio
################################
# src += drivers/fsl_gpio.c

################################
# drivers/fsl_inputmux
################################
# src += drivers/fsl_inputmux.c

################################
# drivers/fsl_pint
################################
# src += drivers/fsl_pint.c

################################
# drivers/fsl_usart
################################
# src += drivers/fsl_usart.c

################################
# drivers/fsl_i2c
################################
# src += drivers/fsl_i2c.c

################################
# drivers/fsl_spi
################################
# src += drivers/fsl_spi.c

################################
# drivers/fsl_flexcomm
################################
# src += drivers/fsl_flexcomm.c

################################
# drivers/fsl_ctimer.h
################################
# src += drivers/fsl_ctimer.c

################################
# drivers/fsl_rtc
################################
# src += drivers/fsl_rtc.c

################################
# drivers/fsl_hashcrypt
################################
# src += drivers/fsl_hashcrypt.c

################################
# drivers/fsl_sdif
################################
# src += drivers/fsl_sdif.c

################################
# components/lists
################################
# includedirs += components/lists
# src += components/lists/fsl_component_generic_list.c
# sdk_config += GENERIC_LIST_LIGHT=0

################################
# components/uart
################################
# includedirs += components/uart
# src += components/uart/fsl_adapter_usart.c

################################
# components/serial_manager
################################
# includedirs += components/serial_manager
# src += components/serial_manager/fsl_component_serial_port_uart.c
# src += components/serial_manager/fsl_component_serial_manager.c

################################
# components/osa
# - components/list
################################
# includedirs += components/osa

## mutually exclusive options
# src += components/osa/fsl_os_abstraction_bm.c
# src += components/osa/fsl_os_abstraction_threadx.c
# src += components/osa/fsl_os_abstraction_free_rtos.c

################################
# utilities/str
################################
# includedirs += utilities/str
# src += utilities/str/fsl_str.c

################################
# debug_console
# - driver/usart
# - drivers/fsl_flexcomm
# - components/lists
# - components/uart
# - components/serial_manager
# - utilities/str
################################
# includedirs += utilities/debug_console
# src += utilities/debug_console/fsl_debug_console.c

# sdk_config += SDK_DEBUGCONSOLE=1
# sdk_config += SDK_DEBUGCONSOLE_UART=1
# sdk_config += SERIAL_PORT_TYPE_UART=1
# sdk_config += DEBUG_CONSOLE_TRANSFER_NON_BLOCKING=1

################################
# sdmmc
# - drivers/fsl_sdif
# - component/osa
################################
# includedirs += sdmmc/common
# includedirs += sdmmc/sd
# includedirs += sdmmc/osa
# includedirs += sdmmc/host/sdif

# src += sdmmc/common/fsl_sdmmc_common.c
# src += sdmmc/sd/fsl_sd.c
# src += sdmmc/osa/fsl_sdmmc_osa.c

## mutually exclusive options
# src += sdmmc/host/sdif/blocking/fsl_sdmmc_host.c
# src += sdmmc/host/sdif/non_blocking/fsl_sdmmc_host.c

# sdk_config += SD_ENABLED

################################
# USB
################################
# includedirs += usb/include
# includedirs += usb/device
# includedirs += usb/phy
# src += usb/device/usb_device_lpcip3511.c
# src += usb/device/usb_device_dci.c

################################
# FatFs
################################
# includedirs += fatfs/source
# includedirs += fatfs/source/fsl_sd_disk
# src += fatfs/source/diskio.c
# src += fatfs/source/ff.c
# src += fatfs/source/ffsystem.c
# src += fatfs/source/ffunicode.c
# src += fatfs/source/fsl_sd_disk/fsl_sd_disk.c

################################
# freeRTOS
################################
### freeRTOS-NTZ
# includedirs += freertos/portable/GCC/ARM_CM33_NTZ/non_secure
# src += freertos/portable/GCC/ARM_CM33_NTZ/non_secure/port.c
# src += freertos/portable/GCC/ARM_CM33_NTZ/non_secure/portasm.c

### freeRTOS-Common
# includedirs += freertos/include
# src += freertos/croutine.c
# src += freertos/event_groups.c
# src += freertos/list.c
# src += freertos/queue.c
# src += freertos/stream_buffer.c
# src += freertos/tasks.c
# src += freertos/timers.c
# src += freertos/portable/MemMang/heap_4.c

# sdk_config += USE_RTOS=1
# sdk_config += SDK_OS_FREE_RTOS

################################
# user
################################
includedirs += board
src += $(wildcard board/*.c)

# includedirs += trustzone
# src += $(wildcard trustzone/*.c)

includedirs += source
src += $(wildcard source/*.c)

################################
# libraries
################################
lib =
lib += ../led_blinky_s/output/cmselib.o

################################################################
# linker_script
################################################################
# linker_script = ld/LPC55S69_cm33_core0_flash.ld
# linker_script = ld/LPC55S69_cm33_core0_flash_s.ld
linker_script = ld/LPC55S69_cm33_core0_flash_ns.ld
