/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/math/Derivatives.h>

namespace ct {
namespace models {
namespace HyQ {

class HyQInverseDynJacReverse : public core::Derivatives<54, 18, double>
{
public:
    typedef Eigen::Matrix<double, 18, 54> JAC_TYPE;
    typedef Eigen::Matrix<double, 54, 1> X_TYPE;

    HyQInverseDynJacReverse()
    {
        jac_.setZero();
        v_.fill(0.0);
    };

    HyQInverseDynJacReverse(const HyQInverseDynJacReverse& other)
    {
        jac_.setZero();
        v_.fill(0.0);
    }

    virtual ~HyQInverseDynJacReverse(){};

    HyQInverseDynJacReverse* clone() const override { return new HyQInverseDynJacReverse(*this); }
    JAC_TYPE jacobian(const Eigen::VectorXd& x_in) override;

private:
    JAC_TYPE jac_;
    std::array<double, 816> v_;
};

} /* namespace HyQ */
} /* namespace models */
} /* namespace ct */
