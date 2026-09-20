#!/bin/sh
# ホストで動く単体試験。実機もSDKも要らない。 sh tests/run.sh
set -e
cd "$(dirname "$0")"
for src in *_test.cpp; do
    out="$(mktemp)"
    g++ -std=c++17 -Wall -Wextra -o "$out" "$src"
    "$out" && echo "PASS $src"
    rm -f "$out"
done
