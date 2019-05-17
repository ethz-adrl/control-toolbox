/*
 * eigenIntegration.h
 *
 *  Created on: Jun 22, 2015
 *      Author: farbod
 */

#pragma once


#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>
#include <boost/version.hpp>


// Necessary routines for Eigen matrices to work with vector_space_algebra
// from odeint
// (that is, it lets odeint treat the eigen matrices correctly, knowing
// how to add, multiply, compute the norm, etc)

namespace Eigen {


#if EIGEN_VERSION_AT_LEAST(3, 3, 0)

// the functions below for Eigen 3.3 unfortunately do not yet return expressions

template <typename D>
inline const CwiseUnaryOp<internal::scalar_abs_op<typename internal::traits<D>::Scalar>, const D> abs(
    const MatrixBase<D>& x)
{
    return x.cwiseAbs();
}


template <typename D>
inline Matrix<typename internal::traits<D>::Scalar, internal::traits<D>::RowsAtCompileTime, 1> operator+(
    const typename internal::traits<D>::Scalar& s,
    const MatrixBase<D>& m)
{
    Matrix<typename internal::traits<D>::Scalar, internal::traits<D>::RowsAtCompileTime, 1> result = m;
    result *= s;
    return result;
}

template <typename D, typename D2>
inline Matrix<typename internal::traits<D>::Scalar, internal::traits<D>::RowsAtCompileTime, 1> operator/(
    const MatrixBase<D>& lhs,
    const MatrixBase<D2>& rhs)
{
    Matrix<typename internal::traits<D>::Scalar, internal::traits<D>::RowsAtCompileTime, 1> res;
    res = (lhs.array() / rhs.array());
    return res;
}


#else


template <typename D>
inline const typename Eigen::
    CwiseUnaryOp<typename Eigen::internal::scalar_add_op<typename Eigen::internal::traits<D>::Scalar>, const D>
    operator+(const typename Eigen::MatrixBase<D>& m, const typename Eigen::internal::traits<D>::Scalar& s)
{
    return Eigen::CwiseUnaryOp<typename Eigen::internal::scalar_add_op<typename Eigen::internal::traits<D>::Scalar>,
        const D>(m.derived(), Eigen::internal::scalar_add_op<typename Eigen::internal::traits<D>::Scalar>(s));
}

template <typename D>
inline const typename Eigen::
    CwiseUnaryOp<typename Eigen::internal::scalar_add_op<typename Eigen::internal::traits<D>::Scalar>, const D>
    operator+(const typename Eigen::internal::traits<D>::Scalar& s, const typename Eigen::MatrixBase<D>& m)
{
    return Eigen::CwiseUnaryOp<typename Eigen::internal::scalar_add_op<typename Eigen::internal::traits<D>::Scalar>,
        const D>(m.derived(), Eigen::internal::scalar_add_op<typename Eigen::internal::traits<D>::Scalar>(s));
}


template <typename D1, typename D2>
inline const typename Eigen::CwiseBinaryOp<
    typename Eigen::internal::scalar_quotient_op<typename Eigen::internal::traits<D1>::Scalar>,
    const D1,
    const D2>
operator/(const Eigen::MatrixBase<D1>& x1, const Eigen::MatrixBase<D2>& x2)
{
    return x1.cwiseQuotient(x2);
}


template <typename D>
inline const typename Eigen::
    CwiseUnaryOp<typename Eigen::internal::scalar_abs_op<typename Eigen::internal::traits<D>::Scalar>, const D>
    abs(const Eigen::MatrixBase<D>& m)
{
    return m.cwiseAbs();
}

#endif


}  // end Eigen namespace


namespace boost {
namespace numeric {
namespace odeint {

#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)

// new boost
template <typename B, int S1, int S2, int O, int M1, int M2>
struct vector_space_norm_inf<Eigen::Matrix<B, S1, S2, O, M1, M2>>
{
    typedef B result_type;

    result_type operator()(const Eigen::Matrix<B, S1, S2, O, M1, M2>& m) const
    {
        return m.template lpNorm<Eigen::Infinity>();
    }
};

#else

// old boost
template <int STATE_DIM, typename SCALAR>
struct vector_space_reduce<Eigen::Matrix<SCALAR, STATE_DIM, 1>>
{
    template <class Op>
    SCALAR operator()(const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x, Op op, SCALAR init) const
    {
        for (int i = 0; i < STATE_DIM; i++)
        {
            init = op(init, x(i));
        }
        return init;
    }
};

#endif
}
}
}  // end boost::numeric::odeint namespace
