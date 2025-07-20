cmake_minimum_required(VERSION 3.10.0)

set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_EXE_LINKER_FLAGS "-pthread")

project(monumental-interview VERSION 0.1.0 LANGUAGES C CXX)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_AUTOMOC ON)

find_package(nlohmann_json 3.12.0 REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Widgets)

include_directories(include lib)

file(GLOB_RECURSE SOURCE_FILES src/*.cpp src/*.h)

add_executable(
    robot-client
    ${SOURCE_FILES}
)

target_link_libraries(robot-client PRIVATE nlohmann_json::nlohmann_json Qt6::Widgets)

include(CTest)
enable_testing()
