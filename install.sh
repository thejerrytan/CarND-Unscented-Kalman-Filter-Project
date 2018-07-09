#!/bin/bash

rm -rf ./build
mkdir build
cmake  -B./build -H./
make -C ./build
