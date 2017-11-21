/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/optcon/optcon-prespec.h>
#include "LqrTest.h"

/*!
 * This runs the LQR unit test.
 * \note for a more straight-forward implementation example, visit the tutorial.
 * \example LqrTest.cpp
 */
int main(int argc, char **argv)
{
    using namespace ct::optcon::example;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
