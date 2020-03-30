
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
template <typename MANIFOLD, typename TAN>
ct::core::ManifoldState<MANIFOLD, TAN> operator+(const ct::core::ManifoldState<MANIFOLD, TAN>& a,
    const ct::core::ManifoldState<MANIFOLD, TAN>& b)
{
    return a.compose(b);
}

template <typename MANIFOLD, typename TAN>
ct::core::ManifoldState<MANIFOLD, TAN> operator*(const double& s, const ct::core::ManifoldState<MANIFOLD, TAN>& m)
{
#ifndef CT_DISABLE_NUMERIC_MANIFOLD_CHECKS  // set this definition for disabling checks
    if (s != typename MANIFOLD::Scalar(1.0))
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
template <typename MANIFOLD, typename TAN>
struct vector_space_norm_inf<ct::core::ManifoldState<MANIFOLD, TAN>>
{
    typedef typename MANIFOLD::Scalar result_type;

    result_type operator()(const ct::core::ManifoldState<MANIFOLD, TAN>& m) const { return m.log().weightedNorm(); }
};

#else

// old boost
template <typename MANIFOLD, typename TAN>
struct vector_space_reduce<ct::core::ManifoldState<MANIFOLD, TAN>>
{
    template <class Op>
    SCALAR operator()(const MANIFOLD& x, Op op, SCALAR init) const
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
