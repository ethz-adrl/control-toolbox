
#pragma once

#ifdef CT_USE_MANIF

#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>
#include <boost/version.hpp>
#include <ct/core/types/ManifoldState.h>

// Necessary routines for manif matrices to work with vector_space_algebra from odeint
// (that is, it lets odeint treat the manif correctly, knowing
// how to add, multiply, compute the norm, etc)
namespace manif {

template <typename MANIFOLD, typename TAN>
ct::core::ManifoldState<MANIFOLD, TAN> operator+(const ct::core::ManifoldState<MANIFOLD, TAN>& a,
    const ct::core::ManifoldState<MANIFOLD, TAN>& b)
{
    return a.compose(b);
}

template <typename MANIFOLD, typename TAN>
ct::core::ManifoldState<MANIFOLD, TAN> operator*(const double& s, const ct::core::ManifoldState<MANIFOLD, TAN>& m)
{
    return (m.log() * s).exp();
}

template <typename TAN>
TAN operator*(const typename internal::traits<TAN>::Scalar& s, const TAN& m)
{
    return m * s;
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
