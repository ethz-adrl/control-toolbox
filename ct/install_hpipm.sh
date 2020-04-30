#!/bin/bash

## get blasfeo
echo "Now installing blasfeo, using a specific commit/tag (!)"
cd /tmp
git clone https://github.com/giaf/blasfeo.git
cd /tmp/blasfeo
git checkout 50dc49e # we currently are on this release
echo -e "\n\n\n======CONFIGURATION STEP NECESSARY======\nBefore continuing, open and modify /tmp/blasfeo/Makefile.rule and set the target to your architecture"
echo "See https://blasfeo.syscop.de/docs/install/#configuration"
echo "Press enter to continue when you are done..."
read a
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