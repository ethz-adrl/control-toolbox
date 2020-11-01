#!/bin/bash
set -x #echo on

## remove existing installation of blasfeo/hpipm
rm -rf /opt/blasfeo
rm -rf /opt/hpipm

## get blasfeo
echo "Now installing blasfeo, using a specific commit/tag (!)"
cd /tmp
rm -rf blasfeo/
git clone https://github.com/giaf/blasfeo.git
cd /tmp/blasfeo
git checkout 0.1.2
mkdir -p build && cd build
cmake ..
make
sudo make install

## get hpipm
echo "Now installing hpipm, using a specific commit/tag (!)"
cd /tmp
rm -rf hpipm/
git clone https://github.com/giaf/hpipm.git
cd /tmp/hpipm
git checkout 0.1.3
mkdir -p build && cd build
cmake ..
make
sudo make install