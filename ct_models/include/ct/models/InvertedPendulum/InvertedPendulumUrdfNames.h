/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace models {
namespace InvertedPendulum {

const std::vector<std::string>& urdfJointNames()
{
    static std::vector<std::string> urdfJointNames{
        "Joint1",
    };

    return urdfJointNames;
}
}
}
}
