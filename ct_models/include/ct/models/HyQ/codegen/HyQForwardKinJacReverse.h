/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/math/Derivatives.h>

namespace ct {
namespace models {
namespace HyQ {

class HyQForwardKinJacReverse : public core::Derivatives<36, 24, double>
{
public:
    typedef Eigen::Matrix<double, 24, 36> JAC_TYPE;
    typedef Eigen::Matrix<double, 36, 1> X_TYPE;

    HyQForwardKinJacReverse()
    {
        jac_.setZero();
        v_.fill(0.0);
    };

    HyQForwardKinJacReverse(const HyQForwardKinJacReverse& other)
    {
        jac_.setZero();
        v_.fill(0.0);
    }

    virtual ~HyQForwardKinJacReverse(){};

    HyQForwardKinJacReverse* clone() const override { return new HyQForwardKinJacReverse(*this); }
    JAC_TYPE jacobian(const Eigen::VectorXd& x_in) override;

private:
    JAC_TYPE jac_;
    std::array<double, 640> v_;
};

} /* namespace HyQ */
} /* namespace models */
} /* namespace ct */
