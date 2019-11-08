cmake_minimum_required(VERSION 3.1)

project(fake_autopilot)

if(NOT MSVC)
    add_definitions("-std=c++11 -Wall -Wextra -Werror")
else()
    add_definitions("-std=c++11 -WX -W2")
endif()

find_package(MAVSDK REQUIRED)

add_executable(fake_autopilot fake_autopilot.cpp)

target_link_libraries(fake_autopilot
    MAVSDK::mavsdk_mavlink_passthrough
    MAVSDK::mavsdk
    )
