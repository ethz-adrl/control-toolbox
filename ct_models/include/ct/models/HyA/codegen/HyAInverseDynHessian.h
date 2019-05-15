/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/math/Derivatives.h>

namespace ct {
namespace models {
namespace HyA {

class HyAInverseDynHessian : public core::Derivatives<12, 6, double>
{
public:
    typedef Eigen::Matrix<double, 12, 12> HES_TYPE;
    typedef Eigen::Matrix<double, 12, 1> X_TYPE;

    HyAInverseDynHessian()
    {
        hessian_.setZero();
        v_.fill(0.0);
    };

    HyAInverseDynHessian(const HyAInverseDynHessian& other)
    {
        hessian_.setZero();
        v_.fill(0.0);
    }

    virtual ~HyAInverseDynHessian(){};

    HyAInverseDynHessian* clone() const override { return new HyAInverseDynHessian(*this); }
    HES_TYPE hessian(const Eigen::VectorXd& x_in, const Eigen::VectorXd& w_in) override;

private:
    HES_TYPE hessian_;
    std::array<double, 351> v_;
};

} /* namespace HyA */
} /* namespace models */
} /* namespace ct */
