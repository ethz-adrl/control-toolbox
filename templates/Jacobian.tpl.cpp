/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

// clang-format off

#include "DERIVATIVE_NAME.h"

namespace ct {
namespace NS1 {
namespace NS2 {


DERIVATIVE_NAME::JAC_TYPE DERIVATIVE_NAME::jacobian(const Eigen::VectorXd& x_in)
{
    double* jac = jac_.data();

    AUTOGENERATED_CODE_PLACEHOLDER

    return jac_;
}

} // namespace NS2
} // namespace NS1
} // namespace ct

// clang-format on