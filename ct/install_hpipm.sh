#!/bin/bash

## get blasfeo
echo "Now installing blasfeo, using a specific commit/tag (!)"
cd /tmp
git clone https://github.com/giaf/blasfeo.git
cd /tmp/blasfeo
git checkout 50dc49e # we currently are on this release
make static_library
sudo make install_static

## get hpipm
echo "Now installing hpipm, using a specific commit/tag (!)"
cd /tmp
git clone https://github.com/giaf/hpipm.git
cd /tmp/hpipm
git checkout 0.1.2
make static_library
make examples
sudo make install_static