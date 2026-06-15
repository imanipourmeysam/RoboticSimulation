cmake_minimum_required(VERSION 3.16)

project(RobotSim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(RobotSim main.cpp
    Joint.h Joint.cpp
    Robot.h Robot.cpp
    Base.h Base.cpp
)

target_include_directories(RobotSim PRIVATE /usr/local/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)


target_link_directories(RobotSim PRIVATE /usr/local/lib)

target_link_libraries(RobotSim raylib m pthread dl)

include(GNUInstallDirs)
install(TARGETS RobotSim
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
