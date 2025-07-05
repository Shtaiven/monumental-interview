cmake_minimum_required(VERSION 3.10.0)
project(monumental-interview VERSION 0.1.0 LANGUAGES C CXX)

add_executable(robot-client src/main.cpp)

include(CTest)
enable_testing()
