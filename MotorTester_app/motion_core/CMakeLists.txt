cmake_minimum_required(VERSION 3.16)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(motion_core LANGUAGES CXX)
endif()

add_library(motion_core STATIC
    src/axis_control_service.cpp
    src/runtime_loop.cpp
    src/config/axis_config_json.cpp
    src/config/hal_runtime_config_json.cpp
)

target_compile_definitions(motion_core PRIVATE MOTION_CORE_WITH_DRIVERS)

target_include_directories(motion_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty
        ${CMAKE_CURRENT_SOURCE_DIR}/../drivers/interfaces
)

target_compile_features(motion_core PUBLIC cxx_std_20)
