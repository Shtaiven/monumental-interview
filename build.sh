#/usr/bin/env bash
# Build the project

mkdir build >/dev/null 2>&1
cd build || exit
if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
    echo "Using conda env to build"
    cmake -DCMAKE_C_COMPILER=x86_64-conda-linux-gnu-gcc -DCMAKE_CXX_COMPILER=x86_64-conda-linux-gnu-g++ -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
else
    cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
fi
make
