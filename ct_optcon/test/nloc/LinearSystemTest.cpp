/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#include <ct/optcon/optcon.h>
#include "LinearSystemTest.h"

/*!
 * This runs the Linear system unit test.
 */
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

    //	ct::optcon::example::singleCore();
    //	ct::optcon::example::multiCore();

    return 1;
}
