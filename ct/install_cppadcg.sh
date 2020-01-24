#!/bin/bash

## get llvm
yes Y | sudo apt-get install llvm

## get clang
# todo

## install CppAD
mkdir /tmp/cppadcg_deps
cd /tmp/cppadcg_deps
wget https://github.com/coin-or/CppAD/archive/20200000.2.tar.gz
tar -xzf 20200000.2.tar.gz
cd CppAD-20200000.2
mkdir build
cd build
cmake -Dcppad_prefix:PATH='/usr/local' ..
sudo make install


## install CppADCodeGen
git clone https://github.com/joaoleal/CppADCodeGen.git /tmp/CppADCodeGen
cd /tmp/CppADCodeGen
git checkout 38d4f3be92810b3f99bcf71da2b05ce45a34e65b  ## commit matching CppAD2020 version
mkdir -p build
cd build
cmake .. #-DLLVM_VERSION=6.0
make
sudo make install
