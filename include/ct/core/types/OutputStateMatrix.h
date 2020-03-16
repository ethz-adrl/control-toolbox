/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

template <size_t OUTPUT_DIM, size_t STATE_DIM, class SCALAR = double>
class OutputStateMatrix : public Eigen::Matrix<SCALAR, OUTPUT_DIM, STATE_DIM>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<SCALAR, OUTPUT_DIM, STATE_DIM> Base;

    OutputStateMatrix() {}
    virtual ~OutputStateMatrix() {}
    //! This constructor allows you to construct MyVectorType from Eigen expressions
    template <typename OtherDerived>
    OutputStateMatrix(const Eigen::MatrixBase<OtherDerived>& other) : Base(other)
    {
    }
    //! This method allows you to assign Eigen expressions to MyVectorType
    template <typename OtherDerived>
    OutputStateMatrix& operator=(const Eigen::MatrixBase<OtherDerived>& other)
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
