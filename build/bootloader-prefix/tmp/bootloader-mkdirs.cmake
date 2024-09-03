# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/cgdo/esp/v5.1.4/esp-idf/components/bootloader/subproject"
  "D:/Works/espidf5/ultrasonic_test/build/bootloader"
  "D:/Works/espidf5/ultrasonic_test/build/bootloader-prefix"
  "D:/Works/espidf5/ultrasonic_test/build/bootloader-prefix/tmp"
  "D:/Works/espidf5/ultrasonic_test/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Works/espidf5/ultrasonic_test/build/bootloader-prefix/src"
  "D:/Works/espidf5/ultrasonic_test/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Works/espidf5/ultrasonic_test/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Works/espidf5/ultrasonic_test/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
