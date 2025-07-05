#/usr/bin/env bash
# Build the project

mkdir build >/dev/null 2>&1
cd build
cmake ..
make
