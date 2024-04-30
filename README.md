# TZ-DataShield (TZDS)

TZ-DataShield is a novel LLVM compiler tool that enhances ARM TrustZone with data-based compartmentalization, offering robust protection for sensitive data against strong adversaries in MCU-based systems.

## Environment

The following environments and hardware were used to perform the experiments for this project:

- Ubuntu 22.04
- [LPCXpresso55S69 development board](https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-/lpcxpresso-boards/lpcxpresso55s69-development-board:LPC55S69-EVK)

![](pictures/BOARD-LPC55S69-EVK-PRODUCT-SHOT.webp)

## Run TZDS

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
