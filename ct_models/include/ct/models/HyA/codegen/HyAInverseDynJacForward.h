/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/math/Derivatives.h>

namespace ct {
namespace models {
namespace HyA {

class HyAInverseDynJacForward : public core::Derivatives<12, 6, double>
{
public:
    typedef Eigen::Matrix<double, 6, 12> JAC_TYPE;
    typedef Eigen::Matrix<double, 12, 1> X_TYPE;

    HyAInverseDynJacForward()
    {
        jac_.setZero();
        v_.fill(0.0);
    };

    HyAInverseDynJacForward(const HyAInverseDynJacForward& other)
    {
        jac_.setZero();
        v_.fill(0.0);
    }

    virtual ~HyAInverseDynJacForward(){};

    HyAInverseDynJacForward* clone() const override { return new HyAInverseDynJacForward(*this); }
    JAC_TYPE jacobian(const Eigen::VectorXd& x_in) override;

private:
    JAC_TYPE jac_;
    std::array<double, 164> v_;
};

} /* namespace HyA */
} /* namespace models */
} /* namespace ct */
