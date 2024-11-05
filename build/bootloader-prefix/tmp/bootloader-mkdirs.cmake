# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/MICROZONE/esp/v5.1.1/esp-idf/components/bootloader/subproject"
  "C:/Users/MICROZONE/Documents/Cypress Projects/New folder/New folder/2.8inch_RGB_LCD_pic/build/bootloader"
  "C:/Users/MICROZONE/Documents/Cypress Projects/New folder/New folder/2.8inch_RGB_LCD_pic/build/bootloader-prefix"
  "C:/Users/MICROZONE/Documents/Cypress Projects/New folder/New folder/2.8inch_RGB_LCD_pic/build/bootloader-prefix/tmp"
  "C:/Users/MICROZONE/Documents/Cypress Projects/New folder/New folder/2.8inch_RGB_LCD_pic/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/MICROZONE/Documents/Cypress Projects/New folder/New folder/2.8inch_RGB_LCD_pic/build/bootloader-prefix/src"
  "C:/Users/MICROZONE/Documents/Cypress Projects/New folder/New folder/2.8inch_RGB_LCD_pic/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/MICROZONE/Documents/Cypress Projects/New folder/New folder/2.8inch_RGB_LCD_pic/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/MICROZONE/Documents/Cypress Projects/New folder/New folder/2.8inch_RGB_LCD_pic/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
