#!/bin/bash

uninstall_module(){
# if module directory exists
if [ -d "../$1" ]; then

  echo "Now uninstalling module " $1 " ... "
  # go to that module
  cd ../$1

  # make and go to build directory
  mkdir -p build && cd build
  sudo make uninstall
  cd ..
else
    echo "ERROR: an error occurred during uninstalling. Try uninstalling manually."
    exit 1
fi
}

## get current folder and make sure it is *ct*
curr_folder=${PWD##*/}
if [ $curr_folder != "ct" ]; then
    echo "ERROR: you need to start the uninstaller from the control-toolbox/ct directory."
    exit 1
fi

uninstall_module ct_doc
uninstall_module ct_models
uninstall_module ct_rbd
uninstall_module ct_optcon
uninstall_module ct_core

exit 0
