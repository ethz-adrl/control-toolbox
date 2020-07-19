#!/bin/bash

## get blasfeo
echo "Now installing blasfeo, using a specific commit/tag (!)"
cd /tmp
git clone https://github.com/giaf/blasfeo.git
cd /tmp/blasfeo
git checkout cc90e14 # we currently are on this commit.
make static_library
sudo make install_static

## get hpipm
echo "Now installing hpipm, using a specific commit/tag (!)"
cd /tmp
git clone https://github.com/giaf/hpipm.git
cd /tmp/hpipm
git checkout 5d9909f # we currently are on this commit.
make static_library
make examples
sudo make install_static