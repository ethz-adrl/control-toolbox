/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <Eigen/Dense>

namespace ct {
namespace core {

template <size_t POS_DIM, size_t VEL_DIM, class SCALAR = double>
class EuclideanStateSymplectic : public Eigen::Matrix<SCALAR, POS_DIM + VEL_DIM, 1>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr size_t PosDim = POS_DIM;
    static constexpr size_t VelDim = VEL_DIM;
    static constexpr size_t TangentDim = POS_DIM + VEL_DIM;
    using Scalar = SCALAR;
    using Tangent = Eigen::Matrix<SCALAR, TangentDim, 1>;
    using PosTangent = Eigen::Matrix<SCALAR, PosDim, 1>;
    using VelTangent = Eigen::Matrix<SCALAR, VelDim, 1>;
    using Base = Eigen::Matrix<SCALAR, POS_DIM + VEL_DIM, 1>;

    static_assert(PosDim > 0 && VelDim > 0, "EuclideanStateSymplectic: dimensions must be greater 0.");

    EuclideanStateSymplectic() = default;
    virtual ~EuclideanStateSymplectic() = default;

    //!This constructor allows you to construct MyVectorType from Eigen expressions
    template <typename OtherDerived>
    EuclideanStateSymplectic(const Eigen::MatrixBase<OtherDerived>& other) : Base(other)
    {
    }

    //! This method allows you to assign Eigen expressions to MyVectorType
    template <typename OtherDerived>
    EuclideanStateSymplectic& operator=(const Eigen::MatrixBase<OtherDerived>& other)
    {
        this->Base::operator=(other);
        return *this;
    }

    static EuclideanStateSymplectic NeutralElement() { return Eigen::Matrix<SCALAR, TangentDim, 1>::Zero(); }
    //! get underlying Eigen type
    Base& toImplementation() { return *this; }
    //! get const underlying Eigen type
    const Base& toImplementation() const { return *this; }
    // provide manifold log operator // TODO: attention - this overloads an eigen function
    const EuclideanStateSymplectic& log() const = delete;  // { return *this; }
    EuclideanStateSymplectic& log() = delete;

    Tangent rminus(const EuclideanStateSymplectic& x) const { return *this - x; }
    EuclideanStateSymplectic rplus(const Tangent& x) const { return *this + x; }
};

} /* namespace core */
} /* namespace ct */
