#!/bin/bash

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
git checkout 806c845
make static_library
make examples
sudo make install_static