#!/bin/bash

build_module(){
# if module directory exists
if [ -d "../$1" ]; then  

  echo " === Starting module " $1 " === "
  # go to that module
  cd ../$1  
  echo "Current directory is " $PWD

  # collect array of all args, remove first arg because that is equal to the module to be built
  build_flags=( "$@" )
  unset build_flags[0]

  # make and go to build directory
  mkdir -p build && cd build

  echo "Building with the following flags ... "
  printf '%s\n' "${build_flags[@]}"
  
  cmake .. ${build_flags[*]}    || { echo "CT cmake failed"; exit 1; } 
  make -j4                      || { echo "CT make failed"; exit 1; }
  make run_tests                || { echo "CT make run_tests failed"; exit 1; }
  sudo make install  >/dev/null
  cd ..
else
    echo "ERROR: an error occurred during building or installing. Try building manually."
    exit 1
fi
}

## get current folder and make sure it is *ct*
curr_folder=${PWD##*/}
if [ $curr_folder != "ct" ]; then
    echo "ERROR: you need to start the installer from the control-toolbox/ct directory."
    exit 1
fi

# check number of user input args
# no args
if [ -z "$1" ] 
  then
    echo "No build flags supplied, using -DCMAKE_BUILD_TYPE=Release"
    BUILD_FLAGS="-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=true"
else 
  # entire user input is interpreted as build flags
  BUILD_FLAGS="-DBUILD_TESTS=true $@"
fi

build_module ct_core $BUILD_FLAGS
build_module ct_optcon $BUILD_FLAGS
build_module ct_rbd $BUILD_FLAGS 
build_module ct_models $BUILD_FLAGS

exit 0
