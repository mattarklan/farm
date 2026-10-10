cmake_minimum_required(VERSION 3.16)
project(RoverApp VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

add_executable(rover_app
    src/main.cpp
    src/debug/debug_server.cpp
    src/motors/serial_port.cpp
    src/motors/motor_interface.cpp
    src/gps/gps_driver.cpp
    src/planning/local_planner.cpp
    src/planning/theta_star.cpp
    src/localization/pose_estimator.cpp
    src/comms/udp_bus.cpp
)

if(WIN32)
    target_link_libraries(rover_app PRIVATE ws2_32)
    target_compile_options(rover_app PRIVATE -Wall -Wextra -Wpedantic -Wno-missing-field-initializers)
endif()