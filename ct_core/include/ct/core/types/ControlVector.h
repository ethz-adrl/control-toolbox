/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <Eigen/Dense>
#include "Constants.h"

namespace ct {
namespace core {

template <class SCALAR = double>
class ControlVector : public Eigen::Matrix<SCALAR, Dynamic, 1>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<SCALAR, Dynamic, 1> Base;

    ControlVector() = default;
    ControlVector(const int d) : Base(d, 1){};

    //! This constructor allows you to construct MyVectorType from Eigen expressions
    template <typename OtherDerived>
    ControlVector(const Eigen::MatrixBase<OtherDerived>& other) : Base(other)
    {
    }

    //! This method allows you to assign Eigen expressions to MyVectorType
    template <typename OtherDerived>
    ControlVector& operator=(const Eigen::MatrixBase<OtherDerived>& other)
    {
        this->Base::operator=(other);
        return *this;
    }

    Base& toImplementation() { return *this; }
    const Base& toImplementation() const { return *this; }
};

using ControlVectord = ControlVector<double>;

} /* namespace core */
} /* namespace ct */
