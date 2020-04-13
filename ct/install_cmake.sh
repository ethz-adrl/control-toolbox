#!/bin/bash

## get latest cmake
echo "removing current cmake ..."
yes Y | apt remove cmake
echo "now installing latest cmake ..."
yes Y | apt install apt-transport-https ca-certificates gnupg software-properties-common wget
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -
DISTRO="$(lsb_release -cs)"
apt-add-repository "deb https://apt.kitware.com/ubuntu/ ${DISTRO} main"
apt update
yes Y | apt install cmake
