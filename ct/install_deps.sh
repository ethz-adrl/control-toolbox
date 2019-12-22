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

## get open mp
yes Y | sudo apt install libomp-dev

## get clang
yes Y | sudo apt install clang

## get CppAD and CppADCodeGen
sudo ./install_cppadcg.sh