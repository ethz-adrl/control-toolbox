/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace models {
namespace HyA {

const std::vector<std::string>& urdfJointNames()
{
    static std::vector<std::string> urdfJointNames{
        "hya_saa_joint", "hya_sfe_joint", "hya_hr_joint", "hya_efe_joint", "hya_wr_joint", "hya_wfe_joint",
    };

    return urdfJointNames;
}
}
}
}
