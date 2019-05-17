// $Id: cppad_eigen.hpp 3757 2015-11-30 12:03:07Z bradbell $
# ifndef CPPAD_CPPAD_EIGEN_HPP
# define CPPAD_CPPAD_EIGEN_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    Eclipse Public License Version 1.0.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */
/*
$begin cppad_eigen.hpp$$
$spell
	impl
	typename
	Real Real
	inline
	neg
	eps
	plugin
	atan
	Num
	acos
	asin
	CppAD
	std::numeric
	enum
	Mul
	Eigen
	cppad.hpp
	namespace
	struct
	typedef
	const
	imag
	sqrt
	exp
	cos
$$
$section Enable Use of Eigen Linear Algebra Package with CppAD$$

$head Syntax$$
$codei%# include <cppad/example/cppad_eigen.hpp>%$$
$children%
	cppad/example/eigen_plugin.hpp%
	example/eigen_array.cpp%
	example/eigen_det.cpp
%$$

$head Purpose$$
Enables the use of the
$href%http://eigen.tuxfamily.org%eigen%$$
linear algebra package with the type $icode%AD<%Base%>%$$.

$head Example$$
The files $cref eigen_array.cpp$$ and $cref eigen_det.cpp$$
contain an example and test of this include file.
It returns true if it succeeds and false otherwise.

$head Include Files$$
The file $code cppad_eigen.hpp$$ includes both
$code <cppad/cppad.hpp>$$ and $code <Eigen/Core>$$.
In addition,
The file $cref eigen_plugin.hpp$$
is used to define $code value_type$$
in the Eigen matrix class definition so its vectors are
$cref/simple vectors/SimpleVector/$$.
$codep */
# define EIGEN_MATRIXBASE_PLUGIN <cppad/example/eigen_plugin.hpp>
# include <Eigen/Core>
# include <cppad/cppad.hpp>
/* $$
$head Eigen NumTraits$$
Eigen needs the following definitions to work properly
with $codei%AD<%Base%>%$$ scalars:
$codep */
namespace Eigen {
	template <class Base> struct NumTraits< CppAD::AD<Base> >
	{	// type that corresponds to the real part of an AD<Base> value
		typedef CppAD::AD<Base>   Real;
		// type for AD<Base> operations that result in non-integer values
		typedef CppAD::AD<Base>   NonInteger;
		// type for nested value inside an AD<Base> expression tree
		typedef CppAD::AD<Base>   Nested;

		typedef CppAD::AD<Base>   Literal;

		enum {
			// does not support complex Base types
			IsComplex             = 0 ,
			// does not support integer Base types
			IsInteger             = 0 ,
			// only support signed Base types
			IsSigned              = 1 ,
			// must initialize an AD<Base> object
			RequireInitialization = 1 ,
			// computational cost of the corresponding operations
			ReadCost              = 1 ,
			AddCost               = 2 ,
			MulCost               = 2
		};

		// machine epsilon with type of real part of x
		// (use assumption that Base is not complex)
		static CppAD::AD<Base> epsilon(void)
		{	return CppAD::numeric_limits< CppAD::AD<Base> >::epsilon(); }

		// relaxed version of machine epsilon for comparison of different
		// operations that should result in the same value
		static CppAD::AD<Base> dummy_precision(void)
		{	return 100. *
				CppAD::numeric_limits< CppAD::AD<Base> >::epsilon();
		}

		// minimum normalized positive value
		static CppAD::AD<Base> lowest(void)
		{	return CppAD::numeric_limits< CppAD::AD<Base> >::min(); }

		// maximum finite value
		static CppAD::AD<Base> highest(void)
		{	return CppAD::numeric_limits< CppAD::AD<Base> >::max(); }

		static const int digits10(void)
		{	return std::numeric_limits<double>::digits10; }


	};
}
/* $$
$head CppAD Namespace$$
Eigen also needs the following definitions to work properly
with $codei%AD<%Base%>%$$ scalars:
$codep */
namespace CppAD {
		// functions that return references
		template <class Base> const AD<Base>& conj(const AD<Base>& x)
		{	return x; }
		template <class Base> const AD<Base>& real(const AD<Base>& x)
		{	return x; }

		// functions that return values (note abs is defined by cppad.hpp)
		template <class Base> AD<Base> imag(const AD<Base>& x)
		{	return CppAD::AD<Base>(0.); }
		template <class Base> AD<Base> abs2(const AD<Base>& x)
		{	return x * x; }
}


#if EIGEN_VERSION_AT_LEAST(3,3,0)
	// no significant decimals needed for Eigen 3.3
#else
namespace Eigen {
	namespace internal {

		template<class Base>
		struct significant_decimals_default_impl< CppAD::AD<Base>, false>
		{	typedef CppAD::AD<Base> Scalar;

			typedef typename NumTraits<Scalar>::Real RealScalar;
			static inline int run()
			{	Scalar neg_log_eps = - log(
					NumTraits<RealScalar>::epsilon()
				);
				int ceil_neg_log_eps = Integer( neg_log_eps );
				if( Scalar(ceil_neg_log_eps) < neg_log_eps )
					ceil_neg_log_eps++;
				return ceil_neg_log_eps;
			}
		};
	}
}
#endif

/* $$
$end
*/
# endif
