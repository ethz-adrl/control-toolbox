/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/math/Derivatives.h>

namespace ct {
namespace models {
namespace HyA {

class HyAInverseDynJacReverse : public core::Derivatives<12, 6, double>
{
public:
    typedef Eigen::Matrix<double, 6, 12> JAC_TYPE;
    typedef Eigen::Matrix<double, 12, 1> X_TYPE;

    HyAInverseDynJacReverse()
    {
        jac_.setZero();
        v_.fill(0.0);
    };

    HyAInverseDynJacReverse(const HyAInverseDynJacReverse& other)
    {
        jac_.setZero();
        v_.fill(0.0);
    }

    virtual ~HyAInverseDynJacReverse(){};

    HyAInverseDynJacReverse* clone() const override { return new HyAInverseDynJacReverse(*this); }
    JAC_TYPE jacobian(const Eigen::VectorXd& x_in) override;

private:
    JAC_TYPE jac_;
    std::array<double, 289> v_;
};

} /* namespace HyA */
} /* namespace models */
} /* namespace ct */
