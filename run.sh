#!/usr/bin/env bash
# Run the project

EXECUTABLE_PATH="build/robot-client"

if [ -f ${EXECUTABLE_PATH} ]; then
    ./${EXECUTABLE_PATH}
else
    echo -e "${EXECUTABLE_PATH} doesn't exist. Try building first with\n\n./build.sh"
fi
