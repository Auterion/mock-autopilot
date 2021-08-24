cmake_minimum_required(VERSION 3.16)

project(mock-autopilot)

if(NOT MSVC)
    add_definitions("-std=c++14 -Wall -Wextra")
else()
    add_definitions("-std=c++14 -WX -W2")
endif()

find_package(MAVSDK REQUIRED)
find_package (Threads)

add_executable(mock-autopilot mock-autopilot.cpp)

target_link_libraries(mock-autopilot
    MAVSDK::mavsdk_mavlink_passthrough
    MAVSDK::mavsdk
)

install(TARGETS mock-autopilot
    RUNTIME DESTINATION bin
)
