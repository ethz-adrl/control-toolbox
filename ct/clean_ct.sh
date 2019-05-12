#!/bin/bash

clean_module(){
# if module directory exists
if [ -d "../$1" ]; then  

  echo "Now cleaning module " $1 " ... "
  # go to that module
  cd ../$1  

  # make and go to build directory
  mkdir -p build && cd build
  rm -r -f *
  cd ..
else
    echo "ERROR: an error occurred during cleaning. Try cleaning manually."
    exit 1
fi
}

## get current folder and make sure it is *ct*
curr_folder=${PWD##*/}
if [ $curr_folder != "ct" ]; then
    echo "ERROR: you need to start the cleaner from the control-toolbox/ct directory."
    exit 1
fi

clean_module ct_core
clean_module ct_optcon
clean_module ct_rbd
clean_module ct_models
clean_module ct_doc

exit 0
