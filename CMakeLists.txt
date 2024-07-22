cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(BaseDevice C CXX ASM)
set(EXECUTABLE BaseDevice)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

set(PICO_BOARD pico CACHE STRING "Board type")

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()


# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_subdirectory(./myomod_interface         myomod_interface)
add_subdirectory(./source                   source          )


add_executable(${EXECUTABLE})

pico_set_program_name(${EXECUTABLE} "${EXECUTABLE}")
pico_set_program_version(${EXECUTABLE} "0.2")

pico_enable_stdio_uart(${EXECUTABLE} 0)
pico_enable_stdio_usb(${EXECUTABLE} 1)

# Add the standard include files to the build
target_include_directories(${EXECUTABLE} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)


# Add any user requested libraries
target_link_libraries(${EXECUTABLE} 
        myomod_interface
        device_source
        )

pico_add_extra_outputs(${EXECUTABLE})

# add linker flags to print cross-reference table in map file and memory usage on console
add_link_options(-Wl,--cref,--print-memory-usage)
# Optional: Print executable size as part of the post build process
add_custom_command(TARGET ${EXECUTABLE}
  POST_BUILD
  COMMAND arm-none-eabi-size ${CMAKE_CURRENT_BINARY_DIR}/${EXECUTABLE}.elf
)