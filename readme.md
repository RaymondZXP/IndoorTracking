# GovTech RTT Indoor Tracking with NRF52833

# Use Guide

Use guide for linux ubuntu system

1. Install GNU make 
    - sudo apt-get install build-essential checkinstall
2. Install GNU toolchain for ARM Cortex-M
    - GNU toolchain including compiler and GDB debugger can be downloaded using this link:
    https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads.
    - Download and install the latest version. Then make sure to add the path to your toolchain to your OS PATH environment variable:
    <path to install directory>/GNU Tools ARM Embedded/4.9 2015q3/bin
    - For ubuntu, you can add the path by:
    sudo nano ~/.bashrc
    go to the last line and add this line:
    export PATH=$PATH: <path to install directory>/gcc-arm-none-eabi-9-2020-q2-update/bin

    - Verify the installation of the toolchain:
    arm-none-eabi-gcc --version

3. Set the toolchain path for the project
    - after cloning the repo, go to lib_sdk/components/toolchain/gcc
    - Modify the content of Makefule.posix:
    GNU_INSTALL_ROOT ?= <path to install directory>/gcc-arm-none-eabi-9-2020-q2-update/bin/  
    (p.s. depends on the version of the toolchain you have installed, please modify the path accordingly)

4. Build the project and flash the DK
    - Navigate to /Master_Anchor/Project/src/pca10100/blank/armgcc
    - Type 'make' in the terminal, GNU make should start buidling using Makefile.
    - Connect one of your NRF52833 DKs
    - type 'make flash' to programme the DK
    - Repeat the same procedure with Slave_Tag folder with another DK, you might be asked which board to flash when you type 'make flash' if you have two boards connected. Check the serial code on the board to identify which board you are flashing.
















