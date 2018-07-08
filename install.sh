#!/bin/bash

rm -rf ./build
mkdir build
cmake  -B./build -H./ -DPYTHON_INCLUDE_DIRS=$(python -c "from sysconfig import get_paths;print(get_paths()['include'])") \
-DPYTHON_LIBRARIES=$(python -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())")
make -C ./build
