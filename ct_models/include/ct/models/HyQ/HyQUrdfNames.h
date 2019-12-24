/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

namespace ct {
namespace models {
namespace HyQ {

static std::vector<std::string> urdfJointNames()
{
    std::vector<std::string> urdfJointNames = {"lf_haa_joint", "lf_hfe_joint", "lf_kfe_joint", "rf_haa_joint",
        "rf_hfe_joint", "rf_kfe_joint", "lh_haa_joint", "lh_hfe_joint", "lh_kfe_joint", "rh_haa_joint", "rh_hfe_joint",
        "rh_kfe_joint"};

    return urdfJointNames;
}

}  // namespace HyQ
}  // namespace models
}  // namespace ct

#pragma GCC diagnostic pop
