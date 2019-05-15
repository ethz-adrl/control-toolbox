/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/core/core-prespec.h>
#include "IntegrationTest.h"


int main(int argc, char** argv)
{
    if (argc > 2)
        plotResult = true;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
