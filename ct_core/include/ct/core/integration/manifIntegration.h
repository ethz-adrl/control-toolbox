
#pragma once

#ifdef CT_USE_MANIF

#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>
#include <boost/version.hpp>
#include <ct/core/types/ManifoldState.h>

/**
 * @brief  Necessary routines for manif matrices to work with vector_space_algebra from boost ODEINT
 * 
 * Lets boost ODEINT steppers treat the manif operations correctly, knowing how to add, multiply, etc.
 * These operations will be used in the CT steppers, as well as in the manif_operations.h header
 */
namespace manif {

// operator overload for adding manifolds
template <template <class> class MANIF_T, template <class> class TAN, typename SCALAR>
ct::core::ManifoldState<MANIF_T, TAN, SCALAR> operator+(const ct::core::ManifoldState<MANIF_T, TAN, SCALAR>& a,
    const ct::core::ManifoldState<MANIF_T, TAN, SCALAR>& b)
{
    return a.compose(b);
}

template <template <class> class MANIF_T, template <class> class TAN, typename SCALAR>
ct::core::ManifoldState<MANIF_T, TAN, SCALAR> operator*(const double& s,
    const ct::core::ManifoldState<MANIF_T, TAN, SCALAR>& m)
{
#ifndef CT_DISABLE_NUMERIC_MANIFOLD_CHECKS  // set this definition for disabling checks
    if (s != SCALAR(1.0))
        throw std::runtime_error("manifold operator* is not defined for scaling. Use different integrator.");
#endif
    return m;
}

template <typename TAN>
TAN operator*(const typename internal::traits<TAN>::Scalar& s, const TAN& t)
{
    return t * s;
}

}  // namespace manif


namespace boost {
namespace numeric {
namespace odeint {

#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)

// new boost
template <template <class> class MANIF_T, template <class> class TAN, typename SCALAR>
struct vector_space_norm_inf<ct::core::ManifoldState<MANIF_T, TAN, SCALAR>>
{
    typedef SCALAR result_type;

    result_type operator()(const ct::core::ManifoldState<MANIF_T, TAN, SCALAR>& m) const
    {
        return m.log().weightedNorm();
    }
};

#else

// old boost
template <template <class> class MANIF_T, template <class> class TAN, typename SCALAR>
struct vector_space_reduce<ct::core::ManifoldState<MANIF_T, TAN, SCALAR>>
{
    template <class Op>
    SCALAR operator()(const MANIF_T<SCALAR>& x, Op op, SCALAR init) const
    {
        auto tan = x.log();

        for (int i = 0; i < tan.coeffs().size(); i++)
        {
            init = op(init, tan.coeffs()(i));
        }

        return init;
    }
};


#endif
}  // namespace odeint
}  // namespace numeric
}  // namespace boost


#endif  // CT_USE_MANIF
