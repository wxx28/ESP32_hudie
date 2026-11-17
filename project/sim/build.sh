#!/usr/bin/env bash
# 编译仿真内核（需要 gcc，例如 MSYS2 mingw64 / Linux / macOS）
set -e
cd "$(dirname "$0")"
FP=../first/first_project/components
gcc -std=c11 -O2 -Wall -Wno-unused-variable \
    -I$FP/Middlewares/algo -I$FP/flight_control -I$FP/flight_control/include -I$FP/BSP/SERVO \
    -Istubs -I../common/hudie_link/include \
    sim.c $FP/Middlewares/algo/*.c $FP/flight_control/fc_output.c $FP/flight_control/fc_config.c \
    ../common/hudie_link/hudie_protocol.c -lm -o hudie_sim
echo "built: $(pwd)/hudie_sim"
