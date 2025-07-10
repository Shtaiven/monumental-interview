cmake_minimum_required(VERSION 3.10.0)

set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_EXE_LINKER_FLAGS "-static -pthread")

project(monumental-interview VERSION 0.1.0 LANGUAGES C CXX)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(nlohmann_json 3.12.0 REQUIRED)

include_directories(include lib)

add_executable(robot-client src/main.cpp)
target_link_libraries(robot-client PRIVATE nlohmann_json::nlohmann_json)

include(CTest)
enable_testing()
