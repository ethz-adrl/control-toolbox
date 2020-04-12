#!/bin/bash

## get cmake 3.14.7
echo "removing system cmake ..."
yes Y | apt remove cmake
echo "now installing cmake 3.14.7 ..."
cd /tmp
wget https://cmake.org/files/v3.14/cmake-3.14.7.tar.gz
tar -xzvf cmake-3.14.7.tar.gz
cd /tmp/cmake-3.14.7
./bootstrap
make
sudo make install
