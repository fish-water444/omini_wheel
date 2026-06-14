cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME omni_wheel)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Application/Chassis_task/chassis_task.c
    Application/DetectTask/detect_task.c
    Bsp/Motor/motor.c
    Bsp/RemoteControl/remote_control.c
    Bsp/MCU/bsp_CAN.c
    Core/Src/bsp_dwt.c
    Core/Src/bsp_usart_idle.c
    Middlewares/Controller/controller.c
    Middlewares/Filter/FirstOrderFilter/filter32.c
    Middlewares/Algorithm/Include/user_lib.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Application/Chassis_task
    Application/Intask
    Application/DetectTask
    Bsp/BMI088
    Bsp/MCU
    Bsp/Motor
    Bsp/RemoteControl
    Components
    Middlewares/Controller
    Middlewares/Filter/FirstOrderFilter
    Middlewares/Filter/KalmanFilter
    Middlewares/Algorithm/Include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM4
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
