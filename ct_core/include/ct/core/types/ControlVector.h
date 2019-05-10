/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

template <int CONTROL_DIM, class SCALAR = double>
class ControlVector : public Eigen::Matrix<SCALAR, CONTROL_DIM, 1>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t DIM = CONTROL_DIM;

    ControlVector(){};
    virtual ~ControlVector(){};

    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, 1> Base;

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

    //! get underlying Eigen type
    Base& toImplementation() { return *this; }
    //! get const underlying Eigen type
    const Base& toImplementation() const { return *this; }
};

} /* namespace core */
} /* namespace ct */
