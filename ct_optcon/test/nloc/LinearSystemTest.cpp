/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#include <ct/optcon/optcon.h>
#include "LinearSystemTest.h"

/*!
 * This runs the Linear system unit test.
 */
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

    //	ct::optcon::example::singleCore();
    //	ct::optcon::example::multiCore();

    return 1;
}
