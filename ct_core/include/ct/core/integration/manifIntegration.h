
#pragma once

#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>
#include <boost/version.hpp>

#include <manif/manif.h>
#include <ct/core/types/ManifoldState.h>
#include <ct/core/types/EuclideanState.h>


// Necessary routines for manif matrices to work with vector_space_algebra from odeint
// (that is, it lets odeint treat the manif correctly, knowing
// how to add, multiply, compute the norm, etc)

namespace ct {
namespace core {

// TOdO: how can we put those into a safe namespace???

template <typename MANIFOLD, typename TAN>
ManifoldState<MANIFOLD, TAN> operator*(const double& s, const ManifoldState<MANIFOLD, TAN>& m)
{
    return (m.log() * s).exp();
}

template <typename MANIFOLD, typename TAN>
ManifoldState<MANIFOLD, TAN> operator+(const ManifoldState<MANIFOLD, TAN>& a, const ManifoldState<MANIFOLD, TAN>& b)
{
    return a.compose(b);
}

manif::SE3Tangentd operator*(const double& s, const typename manif::SE3Tangentd& m)
{
    return m * s;
}

template <typename MANIFOLD, typename TAN>
ManifoldState<MANIFOLD, TAN> abs(const ManifoldState<MANIFOLD, TAN>& p)
{
    // todo
    throw std::runtime_error("calling abs on ManifoldState is currently not supported.");
    return p;
}

template <typename MANIFOLD, typename TAN>
typename ManifoldState<MANIFOLD, TAN>::Tangent abs(const typename ManifoldState<MANIFOLD, TAN>::Tangent& t)
{
    // todo
    throw std::runtime_error("calling abs on ManifoldState Tangent is currently not supported.");
    return t;
}

}  // namespace core
}  // namespace ct


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
