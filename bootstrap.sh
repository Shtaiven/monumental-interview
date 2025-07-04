#!/usr/bin/env bash
# This is an installation script to install pixi, which can then install project dependencies,
# build, and run the program

if command -v curl >/dev/null 2>&1; then
    curl -fsSL https://pixi.sh/install.sh | sh
else
    wget https://pixi.sh/install.sh | sh
fi

source ~/.bashrc
