#!/usr/bin/env bash
# This is an installation script to install pixi, which can then install project dependencies,
# build, and run the program

# Download and install pixi
if command -v curl >/dev/null 2>&1; then
    curl -fsSL https://pixi.sh/install.sh | sh
else
    wget https://pixi.sh/install.sh | sh
fi

# Source the shell
if source ~/.bashrc; then
    if command -v pixi >/dev/null 2>&1; then
        pixi install
        echo -e "Bootstrap successful!"
    else
        echo -e "Didn't find pixi executable. Try opening a new shell in this directory and running\n\npixi install"
    fi
else
    echo -e "Couldn't source ~/.bashrc. Try opening a new shell in this directory and running\n\npixi install"
fi
