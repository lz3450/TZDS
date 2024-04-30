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
# cmse = ns

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
includedirs += CMSIS/Driver/Include
includedirs += LPC55S69
includedirs += drivers
includedirs += cmsis_drivers
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
src += drivers/fsl_gpio.c

################################
# drivers/fsl_inputmux
################################
src += drivers/fsl_inputmux.c

################################
# drivers/fsl_pint
################################
src += drivers/fsl_pint.c

################################
# drivers/fsl_flexcomm
################################
src += drivers/fsl_flexcomm.c

################################
# drivers/fsl_usart
################################
src += drivers/fsl_usart.c

################################
# drivers/fsl_i2c
################################
# src += drivers/fsl_i2c.c

################################
# spi
################################
src += drivers/fsl_spi.c
# src += cmsis_drivers/fsl_spi_cmsis.c

################################
# drivers/fsl_lpadc.h
################################
# src += drivers/fsl_lpadc.c

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
# drivers/fsl_casper
################################
# src += drivers/fsl_casper.c

################################
# drivers/fsl_hashcrypt
################################
# src += drivers/fsl_hashcrypt.c

################################
# drivers/fsl_rng
################################
# src += drivers/fsl_rng.c

################################
# drivers/fsl_hashcrypt
################################
# src += drivers/fsl_hashcrypt.c

################################
# components/lists
################################
includedirs += components/lists
src += components/lists/fsl_component_generic_list.c
sdk_config += GENERIC_LIST_LIGHT=0

################################
# components/uart
################################
includedirs += components/uart
src += components/uart/fsl_adapter_usart.c

################################
# components/serial_manager
################################
includedirs += components/serial_manager
src += components/serial_manager/fsl_component_serial_port_uart.c
src += components/serial_manager/fsl_component_serial_manager.c

################################
# components/mem_manager
################################
# includedirs += components/mem_manager
# src += components/mem_manager/fsl_component_mem_manager.c

# sdk_config += gMemManagerLight=0
# sdk_config += MEM_MANAGER_PRE_CONFIGURE=0
# sdk_config += MEM_MANAGER_ENABLE_TRACE=1

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
includedirs += utilities/str
src += utilities/str/fsl_str.c

################################
# utilities/debug_console
# - driver/fsl_usart
# - drivers/fsl_flexcomm
# - components/lists
# - components/uart
# - components/serial_manager
# - utilities/str
################################
includedirs += utilities/debug_console
src += utilities/debug_console/fsl_debug_console.c

sdk_config += SDK_DEBUGCONSOLE=1
sdk_config += SDK_DEBUGCONSOLE_UART=1
sdk_config += SERIAL_PORT_TYPE_UART=1
# sdk_config += DEBUG_CONSOLE_TRANSFER_NON_BLOCKING=1

################################
# utilities/assert
# - debug_console
################################
src += utilities/fsl_assert.c

################################
# utilities/shell
# - debug_console
################################
# includedirs += utilities/shell
# src += utilities/shell/fsl_shell.c

# sdk_config += SHELL_NON_BLOCKING_MODE=1

################################
# sdmmc
# - drivers/fsl_sdif
# - components/osa
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
# mbedTLS
# - drivers/fsl_casper
# - drivers/fsl_hashcrypt
# - drivers/fsl_rng
################################
# includedirs += mbedtls/include
# includedirs += mbedtls/library
# includedirs += mbedtls/port/ksdk
# src += mbedtls/library/aes.c
# src += mbedtls/library/aesni.c
# src += mbedtls/library/arc4.c
# src += mbedtls/library/aria.c
# src += mbedtls/library/asn1parse.c
# src += mbedtls/library/asn1write.c
# src += mbedtls/library/base64.c
# src += mbedtls/library/bignum.c
# src += mbedtls/library/blowfish.c
# src += mbedtls/library/camellia.c
# src += mbedtls/library/ccm.c
# src += mbedtls/library/certs.c
# src += mbedtls/library/chacha20.c
# src += mbedtls/library/chachapoly.c
# src += mbedtls/library/cipher.c
# src += mbedtls/library/cipher_wrap.c
# src += mbedtls/library/cmac.c
# src += mbedtls/library/constant_time.c
# src += mbedtls/library/ctr_drbg.c
# src += mbedtls/library/debug.c
# src += mbedtls/library/des.c
# src += mbedtls/library/dhm.c
# src += mbedtls/library/ecdh.c
# src += mbedtls/library/ecdsa.c
# src += mbedtls/library/ecjpake.c
# src += mbedtls/library/ecp.c
# src += mbedtls/library/ecp_curves.c
# src += mbedtls/library/entropy.c
# src += mbedtls/library/entropy_poll.c
# src += mbedtls/library/error.c
# src += mbedtls/library/gcm.c
# src += mbedtls/library/havege.c
# src += mbedtls/library/hkdf.c
# src += mbedtls/library/hmac_drbg.c
# src += mbedtls/library/md.c
# src += mbedtls/library/md2.c
# src += mbedtls/library/md4.c
# src += mbedtls/library/md5.c
# src += mbedtls/library/memory_buffer_alloc.c
# src += mbedtls/library/mps_reader.c
# src += mbedtls/library/mps_trace.c
# src += mbedtls/library/net_sockets.c
# src += mbedtls/library/nist_kw.c
# src += mbedtls/library/oid.c
# src += mbedtls/library/padlock.c
# src += mbedtls/library/pem.c
# src += mbedtls/library/pk.c
# src += mbedtls/library/pk_wrap.c
# src += mbedtls/library/pkcs5.c
# src += mbedtls/library/pkcs11.c
# src += mbedtls/library/pkcs12.c
# src += mbedtls/library/pkparse.c
# src += mbedtls/library/pkwrite.c
# src += mbedtls/library/platform.c
# src += mbedtls/library/platform_util.c
# src += mbedtls/library/poly1305.c
# src += mbedtls/library/psa_crypto.c
# src += mbedtls/library/psa_crypto_aead.c
# src += mbedtls/library/psa_crypto_cipher.c
# src += mbedtls/library/psa_crypto_client.c
# src += mbedtls/library/psa_crypto_driver_wrappers.c
# src += mbedtls/library/psa_crypto_ecp.c
# src += mbedtls/library/psa_crypto_hash.c
# src += mbedtls/library/psa_crypto_mac.c
# src += mbedtls/library/psa_crypto_rsa.c
# src += mbedtls/library/psa_crypto_se.c
# src += mbedtls/library/psa_crypto_slot_management.c
# src += mbedtls/library/psa_crypto_storage.c
# src += mbedtls/library/psa_its_file.c
# src += mbedtls/library/ripemd160.c
# src += mbedtls/library/rsa.c
# src += mbedtls/library/rsa_internal.c
# src += mbedtls/library/sha1.c
# src += mbedtls/library/sha256.c
# src += mbedtls/library/sha512.c
# src += mbedtls/library/ssl_cache.c
# src += mbedtls/library/ssl_ciphersuites.c
# src += mbedtls/library/ssl_cli.c
# src += mbedtls/library/ssl_cookie.c
# src += mbedtls/library/ssl_msg.c
# src += mbedtls/library/ssl_srv.c
# src += mbedtls/library/ssl_ticket.c
# src += mbedtls/library/ssl_tls.c
# src += mbedtls/library/ssl_tls13_keys.c
# src += mbedtls/library/threading.c
# src += mbedtls/library/timing.c
# src += mbedtls/library/version.c
# src += mbedtls/library/version_features.c
# src += mbedtls/library/x509.c
# src += mbedtls/library/x509_create.c
# src += mbedtls/library/x509_crl.c
# src += mbedtls/library/x509_crt.c
# src += mbedtls/library/x509_csr.c
# src += mbedtls/library/x509write_crt.c
# src += mbedtls/library/x509write_csr.c
# src += mbedtls/library/xtea.c
# src += mbedtls/port/ksdk/ksdk_mbedtls.c
# src += mbedtls/port/ksdk/des_alt.c
# src += mbedtls/port/ksdk/aes_alt.c
# src += mbedtls/port/ksdk/ecp_alt.c
# src += mbedtls/port/ksdk/ecp_curves_alt.c
# src += mbedtls/port/ksdk/ecp_alt_ksdk.c

# sdk_config += MBEDTLS_CONFIG_FILE=\"ksdk_mbedtls_config.h\"
# sdk_config += FREESCALE_KSDK_BM=1

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
# lib += output/cmselib.o

################################################################
# linker_script
################################################################
linker_script = ld/LPC55S69_cm33_core0_flash.ld
# linker_script = ld/LPC55S69_cm33_core0_flash_s.ld
# linker_script = ld/LPC55S69_cm33_core0_flash_ns.ld
