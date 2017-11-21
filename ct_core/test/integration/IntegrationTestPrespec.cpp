/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/core/core-prespec.h>
#include "IntegrationTest.h"


int main(int argc, char **argv)
{
    if (argc > 2)
        plotResult = true;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
