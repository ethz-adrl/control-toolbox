/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <chrono>

#include <gtest/gtest.h>

#include <ct/optcon/optcon-prespec.h>

#include "SymplecticTest.h"

/*!
 * This runs the GNMS unit test.
 * \note for a more straight-forward implementation example, visit the tutorial.
 * \example GNMSCTest.cpp
 */
int main(int argc, char** argv)
{
    ct::optcon::example::symplecticTest();

    //	testing::InitGoogleTest(&argc, argv);
    //    return RUN_ALL_TESTS();

    //	ct::optcon::example::multiCore();

    return 1;
}
