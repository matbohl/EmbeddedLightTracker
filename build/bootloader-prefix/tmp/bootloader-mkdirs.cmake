# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/mathi/esp/v5.3/esp-idf/components/bootloader/subproject"
  "C:/Users/mathi/Desktop/FH_Technikum/Semester_5/ESP_IDF/EmbeddedLightTracker/build/bootloader"
  "C:/Users/mathi/Desktop/FH_Technikum/Semester_5/ESP_IDF/EmbeddedLightTracker/build/bootloader-prefix"
  "C:/Users/mathi/Desktop/FH_Technikum/Semester_5/ESP_IDF/EmbeddedLightTracker/build/bootloader-prefix/tmp"
  "C:/Users/mathi/Desktop/FH_Technikum/Semester_5/ESP_IDF/EmbeddedLightTracker/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/mathi/Desktop/FH_Technikum/Semester_5/ESP_IDF/EmbeddedLightTracker/build/bootloader-prefix/src"
  "C:/Users/mathi/Desktop/FH_Technikum/Semester_5/ESP_IDF/EmbeddedLightTracker/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/mathi/Desktop/FH_Technikum/Semester_5/ESP_IDF/EmbeddedLightTracker/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/mathi/Desktop/FH_Technikum/Semester_5/ESP_IDF/EmbeddedLightTracker/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
