/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <Eigen/Dense>

namespace ct {
namespace core {

template <int DIM, class SCALAR = double>
class EuclideanState : public Eigen::Matrix<SCALAR, DIM, 1>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr int TangentDim = DIM;
    using Scalar = SCALAR;
    using Tangent = Eigen::Matrix<SCALAR, DIM, 1>;
    using Base = Eigen::Matrix<SCALAR, DIM, 1>;

    EuclideanState() = default;
    virtual ~EuclideanState() = default;

    //!This constructor allows you to construct EuclideanState from Eigen expressions
    template <typename OtherDerived>
    EuclideanState(const Eigen::MatrixBase<OtherDerived>& other) : Base(other)
    {
    }

    //! This method allows you to assign Eigen expressions to EuclideanState
    template <typename OtherDerived>
    EuclideanState& operator=(const Eigen::MatrixBase<OtherDerived>& other)
    {
        this->Base::operator=(other);
        return *this;
    }

    // get EuclideanState templated on a different scalar from this expression
    template <typename OtherScalar>
    using RedefineScalar = EuclideanState<DIM, OtherScalar>;

    static EuclideanState NeutralElement() { return Eigen::Matrix<SCALAR, DIM, 1>::Zero(); }
    //! get underlying Eigen type
    Base& toImplementation() { return *this; }
    //! get const underlying Eigen type
    const Base& toImplementation() const { return *this; }
    // provide manifold log operator // TODO: attention - this overloads an eigen function
    const EuclideanState& log() const = delete;
    EuclideanState& log() = delete;

    Tangent rminus(const EuclideanState& x) const { return *this - x; }
    EuclideanState rplus(const Tangent& x) const { return *this + x; }
};

template <class SCALAR = double>
using EuclideanStateDynamic = EuclideanState<-1, SCALAR>;

using EuclideanStateXd = EuclideanState<-1, double>;

template <int DIM, class SCALAR = double>
using StateVector = EuclideanState<DIM, SCALAR>;  // for legacy, TODO: remove at some point

} /* namespace core */
} /* namespace ct */
