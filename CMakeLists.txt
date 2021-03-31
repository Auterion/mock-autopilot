cmake_minimum_required(VERSION 3.16)

project(fake-autopilot)

if(NOT MSVC)
    add_definitions("-std=c++11 -Wall -Wextra -Werror")
else()
    add_definitions("-std=c++11 -WX -W2")
endif()

find_package(MAVSDK REQUIRED)
find_package (Threads)

add_executable(fake-autopilot fake_autopilot.cpp)

target_link_libraries(fake-autopilot
    MAVSDK::mavsdk_mavlink_passthrough
    MAVSDK::mavsdk
)

install(TARGETS fake-autopilot
    RUNTIME DESTINATION bin
)
