cmake_minimum_required(VERSION 3.16)
project(qt_imu_client LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGL OpenGLWidgets Network)

add_executable(qt_imu_client
    main.cpp
    mainwindow.h
    mainwindow.cpp
    glwidget.h
    glwidget.cpp
    imuworker.h
    imuworker.cpp
)

target_link_libraries(qt_imu_client PRIVATE
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    Qt6::Network
)
