#!/bin/bash

# update and download python and library headers
apt-get update
apt-get install -y python-matplotlib python-numpy python2.7-dev

rm -rf ./build
mkdir build
cmake  -B./build -H./ -DPYTHON_INCLUDE_DIRS=$(python -c "from sysconfig import get_paths;print(get_paths()['include'])") \
-DPYTHON_LIBRARIES=$(python -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())")
make -C ./build
