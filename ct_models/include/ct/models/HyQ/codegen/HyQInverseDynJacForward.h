/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/math/Derivatives.h>

namespace ct {
namespace models {
namespace HyQ {

class HyQInverseDynJacForward : public core::Derivatives<54, 18, double>
{
public:
    typedef Eigen::Matrix<double, 18, 54> JAC_TYPE;
    typedef Eigen::Matrix<double, 54, 1> X_TYPE;

    HyQInverseDynJacForward()
    {
        jac_.setZero();
        v_.fill(0.0);
    };

    HyQInverseDynJacForward(const HyQInverseDynJacForward& other)
    {
        jac_.setZero();
        v_.fill(0.0);
    }

    virtual ~HyQInverseDynJacForward(){};

    HyQInverseDynJacForward* clone() const override { return new HyQInverseDynJacForward(*this); }
    JAC_TYPE jacobian(const Eigen::VectorXd& x_in) override;

private:
    JAC_TYPE jac_;
    std::array<double, 337> v_;
};

} /* namespace HyQ */
} /* namespace models */
} /* namespace ct */
