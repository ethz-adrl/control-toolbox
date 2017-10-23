# Control Toolbox

This is the Control Toolbox v2.0, an efficient C++ library for control, estimation, optimization and motion planning in robotics.

Please find the documentation [here](https://adrlab.bitbucket.io/ct)


Note: in order to leverage the full potential of the ct, enable Eigen Vectorization and multi-threading. Example

$ catkin build <your_package> -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-march=native -mtune=native -mavx -mfma -fopenmp" --force-cmake
