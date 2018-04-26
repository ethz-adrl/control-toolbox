/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/optcon/optcon.h>
#include "SystemInterfaceTest.h"

#include <time.h>

int main(int argc, char **argv)
{
    using namespace ct::optcon::example;
    srand(time(NULL)); // random number generator seed
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
