/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <gtest/gtest.h>
#include <ct/optcon/optcon.h>

#include "ConstrainedLQOCSolverTest.h"

int main(int argc, char** argv)
{
    using namespace ct::optcon;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
