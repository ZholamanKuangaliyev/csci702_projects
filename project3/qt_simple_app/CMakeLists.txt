cmake_minimum_required(VERSION 3.16)
project(qt_simple_app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_executable(qt_simple_app
    main.cpp
    mainwindow.h
    mainwindow.cpp
)

target_link_libraries(qt_simple_app PRIVATE Qt6::Widgets)
