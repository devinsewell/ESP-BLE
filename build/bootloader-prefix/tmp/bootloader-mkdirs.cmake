# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/devinsewell/esp/esp-idf/components/bootloader/subproject"
  "/Users/devinsewell/esp/LuminaSet ESP-BLE/build/bootloader"
  "/Users/devinsewell/esp/LuminaSet ESP-BLE/build/bootloader-prefix"
  "/Users/devinsewell/esp/LuminaSet ESP-BLE/build/bootloader-prefix/tmp"
  "/Users/devinsewell/esp/LuminaSet ESP-BLE/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/devinsewell/esp/LuminaSet ESP-BLE/build/bootloader-prefix/src"
  "/Users/devinsewell/esp/LuminaSet ESP-BLE/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/devinsewell/esp/LuminaSet ESP-BLE/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/devinsewell/esp/LuminaSet ESP-BLE/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
