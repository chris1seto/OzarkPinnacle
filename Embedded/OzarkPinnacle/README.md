# OzarkPinnacle
An Stm32 and DRA818V based APRS node

# Build instructions
(Tested on Ubuntu 20.04)
* $ cd ~/
* $ mkdir opt
* $ cd opt
* $ wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/8-2018q4/gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2
* $ tar -xvfz gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2
* [Add gcc-arm-none-eabi-8-2018-q4-major/bin to your PATH however you see fit]
* $ cd [location of your repo]/OzarkPinnacle/Embedded/OzarkPinnacle/
* $ mkdir build
* $ cd build
* $ cmake ..
* $ make -j4