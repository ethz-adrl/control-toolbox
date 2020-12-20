/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#ifdef CT_USE_MANIF

#include <Eigen/Dense>
#include <manif/manif.h>

namespace ct {
namespace core {

template <template <class> class MANIF_T, template <class> class TAN, typename SCALAR = double>
class ManifoldState : public MANIF_T<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Scalar = SCALAR;
    using Tangent = TAN<SCALAR>;
    static constexpr size_t TangentDim = Tangent::DoF;
    using Base = MANIF_T<SCALAR>;

    ManifoldState();
    virtual ~ManifoldState();

    template <typename OTHER>
    ManifoldState(const OTHER& other);

    // get ManifoldState templated on a different scalar from this expression
    template <typename OtherScalar>
    using RedefineScalar = ManifoldState<MANIF_T, TAN, OtherScalar>;

    static ManifoldState NeutralElement() { return MANIF_T<SCALAR>::Identity(); }
    //! get underlying manif type
    Base& toImplementation() { return *this; }
    //! get const underlying manif type
    const Base& toImplementation() const { return *this; }
};

// NOTE: do not move to implementation file
template <template <class> class MANIF_T, template <class> class TAN, typename SCALAR>
template <typename OTHER>
ManifoldState<MANIF_T, TAN, SCALAR>::ManifoldState(const OTHER& other) : Base(other)
{
}

using SE2d = ManifoldState<manif::SE2, manif::SE2Tangent, double>;
// todo: include other typedefs here.

} /* namespace core */
} /* namespace ct */


#endif  //CT_USE_MANIF