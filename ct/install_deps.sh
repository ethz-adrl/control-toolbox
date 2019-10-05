#!/bin/bash

sudo apt-get update                 

## get lapack
yes Y | sudo apt-get install liblapack-dev

## get eigen3
yes Y | sudo apt-get install libeigen3-dev

## get cmake
yes Y | sudo apt-get install cmake

## get IPOPT
yes Y | sudo apt-get install coinor-libipopt-dev

## get boost
yes Y | sudo apt-get install libboost-all-dev

## get blasfeo 0.1.1
echo "now installing blasfeo 0.1.1 ..."
cd /tmp
git clone https://github.com/giaf/blasfeo.git
cd /tmp/blasfeo
git checkout 0.1.1 # we currently are on this release
make static_library
sudo make install_static

## get hpipm 0.1.1
echo "now installing hpipm 0.1.1 ..."
cd /tmp
git clone https://github.com/giaf/hpipm.git
cd /tmp/hpipm
git checkout 0.1.1
make static_library
make examples
sudo make install_static