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
set(CMAKE_PROJECT_NAME freertos_test)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/APP/bsp_system.h
        Core/APP/scheduler_task.c
        Core/APP/scheduler_task.h
        Core/ringbuffer/ringbuffer.c
        Core/ringbuffer/ringbuffer.h
        Core/task/uart_task.c
        Core/task/uart_task.h
        Core/task/led_task.c
        Core/task/led_task.h
        Core/task/driver/oled.c
        Core/task/driver/oled.h
        Core/task/driver/oledfont.c
        Core/task/driver/oledfont.h
        Core/task/oled_task.c
        Core/task/oled_task.h
        Core/task/key_task.c
        Core/task/key_task.h
        Core/task/driver/delay.c
        Core/task/driver/delay.h
        Core/task/driver/dht11.c
        Core/task/driver/dht11.h
        Core/task/dht11_task.c
        Core/task/dht11_task.h
        Core/task/driver/mq2.c
        Core/task/driver/mq2.h
        Core/task/mq2_task.c
        Core/task/mq2_task.h
        Core/task/driver/eMPL_MPU/dmpKey.h
        Core/task/driver/eMPL_MPU/dmpmap.h
        Core/task/driver/eMPL_MPU/inv_mpu.c
        Core/task/driver/eMPL_MPU/inv_mpu.h
        Core/task/driver/eMPL_MPU/inv_mpu_dmp_motion_driver.c
        Core/task/driver/eMPL_MPU/inv_mpu_dmp_motion_driver.h
        Core/task/mpu6050_task.h
        Core/task/mpu6050_task.c
        Core/task/max30102_task.c
        Core/task/max30102_task.h

)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    MPU6050  # DMP 库需要此定义
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/Core
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx


    # Add user defined libraries

)

# 启用浮点数 printf 支持
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
)
