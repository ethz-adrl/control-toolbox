/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

class IKRegularizerBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IKRegularizerBase() = default;
    virtual ~IKRegularizerBase() = default;

    virtual Eigen::MatrixXd computeRegularizer(const Eigen::VectorXd jointVal) = 0;
};
