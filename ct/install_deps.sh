#!/bin/bash

sudo apt-get update

## get lapack
yes Y | sudo apt-get install liblapack-dev

## get eigen3
yes Y | sudo apt-get install libeigen3-dev

## get cmake
sudo ./install_cmake.sh

## get IPOPT
yes Y | sudo apt-get install coinor-libipopt-dev

## get boost
yes Y | sudo apt-get install libboost-all-dev

## get open mp
yes Y | sudo apt install libomp-dev

## get clang
yes Y | sudo apt install clang

## get python 3 and related python packages
yes Y | sudo apt install python3 python3-dev python3-numpy python3-matplotlib

## get CppAD and CppADCodeGen
sudo ./install_cppadcg.sh
