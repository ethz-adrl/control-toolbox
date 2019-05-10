/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace models {
namespace quadrotor {

static std::vector<std::string> urdfJointNames()
{
    std::vector<std::string> urdfJointNames = {
        "jA", "jB",
    };

    return urdfJointNames;
}

}  // quadrotor
}  // models
}  // ct
