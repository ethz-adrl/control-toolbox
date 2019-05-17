#!/bin/bash

build_doc(){
# if module directory exists
if [ -d "../$1" ]; then  

  echo " === Starting doc for module " $1 " === "
  # go to that module
  cd ../$1  
  echo "Current directory is " $PWD

  # make and go to build directory
  mkdir -p build && cd build
  
  cmake .. 
  make doc
  cd ..
  echo " === completed doc for " $1 "."
else
    echo "ERROR: an error occurred during building the doc."
    exit 1
fi
}

## get current folder and make sure it is *ct*
curr_folder=${PWD##*/}
if [ $curr_folder != "ct" ]; then
    echo "ERROR: you need to start the doc build from the control-toolbox/ct directory."
    exit 1
fi


build_doc ct_core 
build_doc ct_optcon
build_doc ct_rbd
build_doc ct_models
build_doc ct_doc

exit 0
