# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/willa/Desktop/robotic_arm/EmbeddedCode/templates/esp-template/.embuild/espressif/esp-idf/v5.0.2/components/bootloader/subproject"
  "/Users/willa/Desktop/robotic_arm/EmbeddedCode/templates/esp-template/target/riscv32imc-esp-espidf/debug/build/esp-idf-sys-9f6b4ed75c495505/out/build/bootloader"
  "/Users/willa/Desktop/robotic_arm/EmbeddedCode/templates/esp-template/target/riscv32imc-esp-espidf/debug/build/esp-idf-sys-9f6b4ed75c495505/out/build/bootloader-prefix"
  "/Users/willa/Desktop/robotic_arm/EmbeddedCode/templates/esp-template/target/riscv32imc-esp-espidf/debug/build/esp-idf-sys-9f6b4ed75c495505/out/build/bootloader-prefix/tmp"
  "/Users/willa/Desktop/robotic_arm/EmbeddedCode/templates/esp-template/target/riscv32imc-esp-espidf/debug/build/esp-idf-sys-9f6b4ed75c495505/out/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/willa/Desktop/robotic_arm/EmbeddedCode/templates/esp-template/target/riscv32imc-esp-espidf/debug/build/esp-idf-sys-9f6b4ed75c495505/out/build/bootloader-prefix/src"
  "/Users/willa/Desktop/robotic_arm/EmbeddedCode/templates/esp-template/target/riscv32imc-esp-espidf/debug/build/esp-idf-sys-9f6b4ed75c495505/out/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/willa/Desktop/robotic_arm/EmbeddedCode/templates/esp-template/target/riscv32imc-esp-espidf/debug/build/esp-idf-sys-9f6b4ed75c495505/out/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/willa/Desktop/robotic_arm/EmbeddedCode/templates/esp-template/target/riscv32imc-esp-espidf/debug/build/esp-idf-sys-9f6b4ed75c495505/out/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
