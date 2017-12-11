/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace models {
namespace HyA {

static std::vector<double> jointLowerLimit()
{
    std::vector<double> jointLowerLimit = {-3.1416, -0.7679, -1.6406, 0, -2.0944, -0.5236};

    return jointLowerLimit;
}

static std::vector<double> jointUpperLimit()
{
    std::vector<double> jointUpperLimit = {0.5236, 0.8552, 0.0698, 2.2689, 1.5708, 1.5708};

    return jointUpperLimit;
}

static std::vector<double> jointVelocityLimit()
{
    std::vector<double> jointVelocityLimit = {12.0, 12.0, 12.0, 12.0, 12.0, 12.0};

    return jointVelocityLimit;
}
}
}
}
