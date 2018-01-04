/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <gtest/gtest.h>
#include <ct/optcon/optcon.h>

#include "ConstrainedLQOCSolverTest.h"

int main(int argc, char **argv)
{
    using namespace ct::optcon;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
