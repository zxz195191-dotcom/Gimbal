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
set(CMAKE_PROJECT_NAME G3508_Test)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        # -------- PID ----------
        Core/Application/Algorithm/pid.c

        # -------- App 应用层 -----
        Core/Application/App/app.c

        # -------- Devices 硬件底层：电机、CAN底层 --------
        Core/Application/Devices/Motor/motor.c
        Core/Application/Devices/Motor/dm_motor.c
        Core/Application/Devices/Can_Dispatch/can_dispatch.c
        Core/Application/Devices/Vision/vision_protocol.c

        # -------- Module 机构业务：云台、发射、安全 --------
        Core/Application/Module/Gimbal/gimbal.c
        Core/Application/Module/Shooter/shooter.c
        Core/Application/Module/Saftery/saftery.c
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
    ${CMAKE_SOURCE_DIR}/Core/Application
    ${CMAKE_SOURCE_DIR}/Core/Application/Algorithm
    ${CMAKE_SOURCE_DIR}/Core/Application/App

    # Devices
    ${CMAKE_SOURCE_DIR}/Core/Application/Devices
    ${CMAKE_SOURCE_DIR}/Core/Application/Devices/Motor
    ${CMAKE_SOURCE_DIR}/Core/Application/Devices/Can_Dispatch
    ${CMAKE_SOURCE_DIR}/Core/Application/Devices/Vision

    # Module
    ${CMAKE_SOURCE_DIR}/Core/Application/Module
    ${CMAKE_SOURCE_DIR}/Core/Application/Module/Gimbal
    ${CMAKE_SOURCE_DIR}/Core/Application/Module/Shooter
    ${CMAKE_SOURCE_DIR}/Core/Application/Module/Saftery
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
