# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Guilherme/esp/esp-idf/components/bootloader/subproject"
  "X:/Stuffs/Faculdade/2022.1/ie/projects/uc/blink/build/bootloader"
  "X:/Stuffs/Faculdade/2022.1/ie/projects/uc/blink/build/bootloader-prefix"
  "X:/Stuffs/Faculdade/2022.1/ie/projects/uc/blink/build/bootloader-prefix/tmp"
  "X:/Stuffs/Faculdade/2022.1/ie/projects/uc/blink/build/bootloader-prefix/src/bootloader-stamp"
  "X:/Stuffs/Faculdade/2022.1/ie/projects/uc/blink/build/bootloader-prefix/src"
  "X:/Stuffs/Faculdade/2022.1/ie/projects/uc/blink/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "X:/Stuffs/Faculdade/2022.1/ie/projects/uc/blink/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "X:/Stuffs/Faculdade/2022.1/ie/projects/uc/blink/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
