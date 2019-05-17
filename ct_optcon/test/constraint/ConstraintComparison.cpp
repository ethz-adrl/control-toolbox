/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#define EIGEN_INITIALIZE_MATRICES_BY_NAN

#include <ct/optcon/optcon.h>
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "ConstraintComparison.h"

using namespace ct::optcon::example;

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
