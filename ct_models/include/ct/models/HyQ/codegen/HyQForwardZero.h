/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/math/Derivatives.h>

namespace ct {
namespace models {
namespace HyQ {

class HyQForwardZero : public core::Derivatives<49, 36, double>
{
public:
    typedef Eigen::Matrix<double, 36, 1> OUT_TYPE;
    typedef Eigen::Matrix<double, 49, 1> X_TYPE;

    HyQForwardZero()
    {
        eval_.setZero();
        v_.fill(0.0);
    };

    HyQForwardZero(const HyQForwardZero& other)
    {
        eval_.setZero();
        v_.fill(0.0);
    }

    virtual ~HyQForwardZero(){};

    HyQForwardZero* clone() const override { return new HyQForwardZero(*this); }
    OUT_TYPE forwardZero(const Eigen::VectorXd& x_in) override;

private:
    OUT_TYPE eval_;
    std::array<double, 402> v_;
};

} /* namespace HyQ */
} /* namespace models */
} /* namespace ct */
