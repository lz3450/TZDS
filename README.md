# TZ-DataShield (TZDS)

TZ-DataShield is a novel LLVM compiler tool that enhances ARM TrustZone with data-based compartmentalization, offering robust protection for sensitive data against strong adversaries in MCU-based systems.

## Environment

The following environments and hardware were used to perform the experiments for this project:

- Ubuntu 22.04
- [LPCXpresso55S69 development board](https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-/lpcxpresso-boards/lpcxpresso55s69-development-board:LPC55S69-EVK)

![](pictures/BOARD-LPC55S69-EVK-PRODUCT-SHOT.webp)

## Artifact Evaluation

**Note: After the AE evaluation, we will open-source our work through our gitlab public directory** [https://gitlab.com/s3lab-code/public](https://gitlab.com/s3lab-code/public)

### Connect to Remote Server

```sh
ssh -p 2222 tzds@107.211.15.224
```
(See password in *Artifact Appendix*)

### Experiment 1

[3human-minutes + 10 compute-minutes]

Sensitive Data Flow Identification

Open a new terminal and navigate to one application directory, for example, `adc`.

```sh
cd lpc_firmware
# use adc as example
cd adc
# install/update template code (including sdk)
./install-template.sh
make clean && make COMPILER=clang -j $(nproc)
make sdf
cat output/comp.yaml
```

The `.yaml` file shows the compartments, peripheral and shared global variables.

```
~/TZDS/lpc_firmware/adc$ make COMPILER=clang -j16
AS        startup/startup_LPC55S69_cm33_core0.S
CC        source/sys_tick.c
CC        source/main.c
CC        board/pin_mux.c
CC        board/peripherals.c
CC        board/clock_config.c
CC        utilities/debug_console/fsl_debug_console.c
CC        utilities/str/fsl_str.c
CC        components/serial_manager/fsl_component_serial_manager.c
CC        components/serial_manager/fsl_component_serial_port_uart.c
CC        components/uart/fsl_adapter_usart.c
CC        components/lists/fsl_component_generic_list.c
CC        drivers/fsl_lpadc.c
CC        drivers/fsl_usart.c
CC        drivers/fsl_flexcomm.c
CC        drivers/fsl_gpio.c
CC        drivers/fsl_reset.c
CC        drivers/fsl_power.c
CC        drivers/fsl_clock.c
CC        drivers/fsl_common_arm.c
CC        drivers/fsl_common.c
CC        LPC55S69/system_LPC55S69_cm33_core0.c
ASLL      .ir/source/sys_tick.c.ll
ASLL      .ir/source/main.c.ll
ASLL      .ir/board/pin_mux.c.ll
ASLL      .ir/board/peripherals.c.ll
ASLL      .ir/board/clock_config.c.ll
ASLL      .ir/components/serial_manager/fsl_component_serial_manager.c.ll
ASLL      .ir/components/serial_manager/fsl_component_serial_port_uart.c.ll
ASLL      .ir/components/uart/fsl_adapter_usart.c.ll
ASLL      .ir/components/lists/fsl_component_generic_list.c.ll
ASLL      .ir/drivers/fsl_flexcomm.c.ll
ASLL      .ir/drivers/fsl_gpio.c.ll
ASLL      .ir/drivers/fsl_reset.c.ll
ASLL      .ir/utilities/debug_console/fsl_debug_console.c.ll
ASLL      .ir/drivers/fsl_lpadc.c.ll
ASLL      .ir/utilities/str/fsl_str.c.ll
ASLL      .ir/drivers/fsl_usart.c.ll
ASLL      .ir/drivers/fsl_common_arm.c.ll
ASLL      .ir/LPC55S69/system_LPC55S69_cm33_core0.c.ll
ASLL      .ir/drivers/fsl_common.c.ll
ASLL      .ir/drivers/fsl_power.c.ll
ASLL      .ir/drivers/fsl_clock.c.ll
LLVMLINK  output/adc.ll
LLVMLINK  output/adc.bc
LD        output/adc.elf
GEN       compile_commands.json
Memory region         Used Size  Region Size  %age Used
    m_interrupts:         304 B        512 B     59.38%
          m_text:       41568 B     466432 B      8.91%
   m_core1_image:          0 GB       174 KB      0.00%
          m_data:        5840 B       204 KB      2.80%
    rpmsg_sh_mem:          0 GB         0 GB
      m_usb_sram:          0 GB        16 KB      0.00%
GEN       output/adc.hex
GEN       output/adc.bin
```

```
~/TZDS/lpc_firmware/adc$ cat output/comp.yaml
base:
  DbgConsole_Init, FLEXCOMM_GetInstance, FLEXCOMM_Init, FLEXCOMM_PeripheralIsPresent, FLEXCOMM_SetPeriph, HAL_UartGetStatus, HAL_UartInit, HAL_UartInitCommon, SerialManager_Init, SerialManager_OpenWriteHandle, Serial_UartInit, USART_EnableContinuousSCLK, USART_GetDefaultConfig, USART_Init, USART_SetBaudRate:
    - g_serialHandle
    - USART
  DbgConsole_Printf, ConvertFloatRadixNumToString, ConvertPrecisionWidthToLength, ConvertRadixNumToString, DbgConsole_PrintCallback, DbgConsole_SendDataReliable, DbgConsole_Vprintf, HAL_UartSendBlocking, PrintCheckFlags, PrintGetLengthFlag, PrintGetPrecision, PrintGetRadixFromobpu, PrintGetSignChar, PrintGetWidth, PrintIsdi, PrintIsfF, PrintIsobpu, PrintIsxX, PrintOutputdifFobpu, PrintOutputxX, SerialManager_StartWriting, SerialManager_Write, SerialManager_WriteBlocking, Serial_UartWrite, StrFormatExaminedi, StrFormatExamineobpu, StrFormatExaminexX, StrFormatPrintf, USART_WriteBlocking, modf, pow:
    - g_SensorData, g_serialHandle
    - USART
  LPADC_Init, LPADC_DoResetConfig, LPADC_DoResetFIFO0, LPADC_DoResetFIFO1, LPADC_Enable, LPADC_GetInstance:
    - ADC0_config
    - ADC
  LPADC_DoAutoCalibration, LPADC_FinishAutoCalibration, LPADC_GetConvResultCount, LPADC_GetGainConvResult, LPADC_PrepareAutoCalibration:
    -
    - ADC
  LPADC_SetConvCommandConfig:
    - ADC0_commandsConfig
    - ADC
  LPADC_SetConvTriggerConfig:
    - ADC0_triggersConfig
    - ADC
  LPADC_SetOffsetValue:
    -
    - ADC
  LPADC_EnableInterrupts:
    -
    - ADC
  LPADC_DoSoftwareTrigger:
    -
    - ADC
  MeasureTemperature:
    - g_SensorData
    -
  ADC0_IRQHandler, LPADC_GetConvResult:
    - g_SensorData, g_LpadcConversionCompletedFlag
    - ADC
joint:
  ReadTemperature:
    - g_LpadcConversionCompletedFlag
    - ADC
memory:
  peripheral:
    - USART
    - ADC
  marked:
    - g_SensorData
    - g_LpadcConversionCompletedFlag
  identified:
    - ADC0_config
    - ADC0_commandsConfig
    - ADC0_triggersConfig
    - g_serialHandle
  shared:
    - g_LpadcConversionCompletedFlag
    - g_SensorData
    - g_serialHandle
```

### Experiment 2

5 human-minutes + 20 compute-minutes

Compartmentalized Firmware Generation

Open a new terminal and navigate to one application directory, for example, `adc`.

```sh
cd lpc_firmware
# use adc as example
cd adc
# install/update template code (including sdk)
./install-template.sh
make clean && make COMPILER=clang -j $(nproc)
ls -l output/adc.hex
```

### Experiment 3

10 human-minutes

Download Firmware Images to MCU

Open a new terminal and navigate to one application directory, for example, `adc`. Then, open another terminal and run minicom-c on-b 115200-D /dev/ttyACM0 to open a serial terminal connected to the MCU.

#### Terminal 1:
```sh
cd lpc_firmware
# use adc as example
cd adc
# install/update template code (including sdk)
./install-template.sh
make clean && make COMPILER=clang -j $(nproc)
make download
```

#### Terminal 2:
```sh
minicom -c on -b 115200 -D /dev/ttyACM0

Welcome to minicom 2.8

OPTIONS: I18n
Port /dev/ttyACM0, 11:42:50

Press CTRL-A Z for help on special keys

Current temperature:  28.44
Current temperature:  28.47
Current temperature:  28.89
Current temperature:  28.47
...
```

---
### Experiment 3 for `usbvcom` & `freertos_usbvcom`

For `usbvcom` and `freertos_usbvcom`, a third terminal is necessary to get proper output of the application.

#### Terminal 1:
```sh
cd lpc_firmware
cd usbvcom
## or
# cd freertos_usbvcom
./install-template.sh
make clean && make COMPILER=clang -j $(nproc)
make download
```

#### Terminal 2:
```sh
minicom -c on -b 115200 -D /dev/ttyACM0
```

#### Terminal 3:
```sh
ls /dev/ttyACM*
# You will see:
# /dev/ttyACM0  /dev/ttyACM1
# Then, open another minicom to connect /dev/ttyACM1
minicom -c on -b 115200 -D /dev/ttyACM1
# In the minicom, type random characters, you can observe that the terminal echos the characters you type
# In terminal 2, you can also see the typed characters
```

### Experiment 4

20 human-minutes + 10 compute-minutes

Over-approximation and Under approximation Rate

This experiment aims to measure the accuracy of TZ-DATASHIELD’s static analysis tool by using well-understood C/C++ programs with known data slices.

```sh
cd slicing_benchmark/1
./compile.sh
ls
```
You will see:
```
1.c   1.svf.bc       compile.sh     groundtruth.png  icfg.dot  vfg.dot
1.ll  callgraph.dot  full_svfg.dot  groundtruth.svg  pag.dot   vfg_model.dot
```
Now you can compare the ground truth value flow graph (`groundtruth.svg`) and the generated value flow graph (`vfg_model.dot`, using a online tool, like https://dreampuf.github.io/).

### Experiment 5

SFI, CFI, and DFI Enforcement

This experiment aims to check if the SFI, CFI, and DFI mechanisms can block illegal access to global variables and peripherals. The attacks are implemented as maliciously modified compartments.

**Terminal 1**:
```sh
cd attacks/1
make download
```

**Terminal 2**:
```sh
minicom -c on -b 115200 -D /dev/ttyACM0
```

```
~/TZDS/attacks/adc$ minicom -c on -b 115200 -D /dev/ttyACM0

Welcome to minicom 2.8

OPTIONS: I18n
Port /dev/ttyACM0

Press CTRL-A Z for help on special keys

SFI violation detected!
```

## Evaluation Results Explanation

### Table III, IV, V, VII and Figure 4, 5, 7

The data presented in the above tables and figures are calculated through static measurement of the compiled firmware.
The original data measured are placed under `data-source/<app_name>` and the preprocessed data are placed under `data/<app_name>`, which are precessed through Pyhton script `preprocess.py`.
The following section explains the relation ship between tables/figures and the data measured.
We mainly use Excel (`data-ndss2025.xlsx`) to calculate the results shown in the tables and figures.

#### Table III

- **# C & # Fn**: `lpc_firmware/<app_name>/output/comp.yaml`
- **Size (KB)**: `data/<app_name>/function_sizes.json`

#### Table IV

- **#**: `lpc_firmware/<app_name>/output/comp.yaml`
- **Size**: `data/<app_name>/object_sizes.json`

#### Table V, Table VII, Figure 4 and Figure 7

- `data/<app_name>/object_sizes.json`
- `data/<app_name>/function_sizes.json`
- `data/<app_name>/sizes.json`

#### Figure 5

- `data/<app_name>/rop_pos.json`
- `data-source/<app_name>/rop.txt`

### Table VI and Figure 6, 8

The data presented in the above tables and figures are calculated by measuring the running application. We the Data Watchpoint and Trace (DWT) Cycle Counter in the MCU to measure the time used to run a piece of code:

```c
uint32_t start, end, cycles;
start = DWT->CYCCNT; // Get start cycle count
// Code to measure
...
end = DWT->CYCCNT
cycles = end - start // Calculate elapsed cycles
```

The measured time are processed using Excel `data-ndss2025.xlsx`.

#### Table VI and Figure 8

- **SFI/CFI/DFI**:
  - Measure the time used to execute `check()` function of the security monitor.
  - Count the added instructions after instrumentation and calculate their execution time according to instruction type.

- **Compartment Swith/Load/Unload**: Measure the time used by the  corresponding functions of the security monitor.

#### Figure 6

Some compartments are only used for the initialization of sensitive peripherals, and they are invoked just once during the lifetime of the application, while others are repeatedly executed within the application loop. So the runtime overhead are only measured and reported on those repeatedly called compartments. For application that requires human input (such as `pinlock` and `usbvcom`), the time wait for the input are exempted out.

---

## Run TZDS from scratch

### Build toolchain

#### Build gcc (>30 minutes)
```sh
sudo apt update
sudo apt install build-essential
cd toolchain-gnu-bare-metal
./build.sh
```

#### Build llvm and clang (>20 minutes)
```sh
sudo apt install clang ninja-build
cd LLVM-embedded-toolchain-for-Arm
./build.sh
```

#### Build SVF (>5 minutes)
```sh
cd SVF
./build.sh
```

### Install toolchain
```sh
tar -xf toolchain-gnu-bare-metal/LLVMEmbeddedToolchainForArm-branch-14.tar.gz -C toolchain --strip-components=1
tar -xf LLVM-embedded-toolchain-for-Arm/LLVMEmbeddedToolchainForArm-branch-14.tar.gz -C toolchain --strip-components=1
```

### Install JLink udev rule

- Copy the file "99-jlink.rules" provided with this software package
  in the /etc/udev/rules.d/ directory using this command:
```sh
sudo cp 99-jlink.rules /etc/udev/rules.d/
```

- Either restart your system or manually trigger the new rules with the following commands:
```sh
sudo udevadm control -R
sudo udevadm trigger --action=remove --attr-match=idVendor=1366 --subsystem-match=usb
sudo udevadm trigger --action=add    --attr-match=idVendor=1366 --subsystem-match=usb
```

### Build firmware

- Connect the debug usb port of development board
```sh
cd lpc_firmware
# use i2c_sensor as example
cd i2c_sensor
# install/update template code (including sdk) from template project
./install-template.sh
make COMPILER=clang -j $(nproc)
```

- This command will generate the analysis results and and firmware:
    - original firmware
    - normal world firmware
    - secure world firmware
    - analysis results

Note that the normal world firmware and secure world firmware should be programed to the MCU together to work.

Sample data generate for compartmentalization can be find in `data-source` in this repo.
