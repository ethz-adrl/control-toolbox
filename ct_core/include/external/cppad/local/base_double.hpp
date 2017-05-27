// $Id: base_double.hpp 3769 2015-12-29 16:13:16Z bradbell $
# ifndef CPPAD_BASE_DOUBLE_HPP
# define CPPAD_BASE_DOUBLE_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    Eclipse Public License Version 1.0.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */
# include <cppad/configure.hpp>
# include <limits>

/*
$begin base_double.hpp$$
$spell
	cppad
	hpp
	azmul
	expm1
	atanh
	acosh
	asinh
	erf
	endif
	abs_geq
	acos
	asin
	atan
	cos
	sqrt
	tanh
	std
	fabs
	bool
	Lt Le Eq Ge Gt
	Rel
	CppAD
	CondExpOp
	namespace
	inline
	enum
	const
	exp
	const
$$


$section Enable use of AD<Base> where Base is double$$

$head CondExpOp$$
The type $code double$$ is a relatively simple type that supports
$code <$$, $code <=$$, $code ==$$, $code >=$$, and $code >$$ operators; see
$cref/ordered type/base_cond_exp/CondExpTemplate/Ordered Type/$$.
Hence its $code CondExpOp$$ function is defined by
$codep */
namespace CppAD {
	inline double CondExpOp(
		enum CompareOp     cop          ,
		const double&       left         ,
		const double&       right        ,
		const double&       exp_if_true  ,
		const double&       exp_if_false )
	{	return CondExpTemplate(cop, left, right, exp_if_true, exp_if_false);
	}
}
/* $$

$head CondExpRel$$
The $cref/CPPAD_COND_EXP_REL/base_cond_exp/CondExpRel/$$ macro invocation
$codep */
namespace CppAD {
	CPPAD_COND_EXP_REL(double)
}
/* $$
uses $code CondExpOp$$ above to
define $codei%CondExp%Rel%$$ for $code double$$ arguments
and $icode%Rel%$$ equal to
$code Lt$$, $code Le$$, $code Eq$$, $code Ge$$, and $code Gt$$.

$head EqualOpSeq$$
The type $code double$$ is simple (in this respect) and so we define
$codep */
namespace CppAD {
	inline bool EqualOpSeq(const double& x, const double& y)
	{	return x == y; }
}
/* $$

$head Identical$$
The type $code double$$ is simple (in this respect) and so we define
$codep */
namespace CppAD {
	inline bool IdenticalPar(const double& x)
	{	return true; }
	inline bool IdenticalZero(const double& x)
	{	return (x == 0.); }
	inline bool IdenticalOne(const double& x)
	{	return (x == 1.); }
	inline bool IdenticalEqualPar(const double& x, const double& y)
	{	return (x == y); }
}
/* $$

$head Integer$$
$codep */
namespace CppAD {
	inline int Integer(const double& x)
	{	return static_cast<int>(x); }
}
/* $$

$head azmul$$
$codep */
namespace CppAD {
	CPPAD_AZMUL( double )
}
/* $$

$head Ordered$$
The $code double$$ type supports ordered comparisons
$codep */
namespace CppAD {
	inline bool GreaterThanZero(const double& x)
	{	return x > 0.; }
	inline bool GreaterThanOrZero(const double& x)
	{	return x >= 0.; }
	inline bool LessThanZero(const double& x)
	{	return x < 0.; }
	inline bool LessThanOrZero(const double& x)
	{	return x <= 0.; }
	inline bool abs_geq(const double& x, const double& y)
	{	return std::fabs(x) >= std::fabs(y); }
}
/* $$

$head Unary Standard Math$$
The following macro invocations define the unary standard math functions
required to use $code AD<double>$$:
$codep */
namespace CppAD {
	CPPAD_STANDARD_MATH_UNARY(double, acos)
	CPPAD_STANDARD_MATH_UNARY(double, asin)
	CPPAD_STANDARD_MATH_UNARY(double, atan)
	CPPAD_STANDARD_MATH_UNARY(double, cos)
	CPPAD_STANDARD_MATH_UNARY(double, cosh)
	CPPAD_STANDARD_MATH_UNARY(double, exp)
	CPPAD_STANDARD_MATH_UNARY(double, fabs)
	CPPAD_STANDARD_MATH_UNARY(double, log)
	CPPAD_STANDARD_MATH_UNARY(double, log10)
	CPPAD_STANDARD_MATH_UNARY(double, sin)
	CPPAD_STANDARD_MATH_UNARY(double, sinh)
	CPPAD_STANDARD_MATH_UNARY(double, sqrt)
	CPPAD_STANDARD_MATH_UNARY(double, tan)
	CPPAD_STANDARD_MATH_UNARY(double, tanh)
# if CPPAD_USE_CPLUSPLUS_2011
	CPPAD_STANDARD_MATH_UNARY(double, erf)
	CPPAD_STANDARD_MATH_UNARY(double, asinh)
	CPPAD_STANDARD_MATH_UNARY(double, acosh)
	CPPAD_STANDARD_MATH_UNARY(double, atanh)
	CPPAD_STANDARD_MATH_UNARY(double, expm1)
	CPPAD_STANDARD_MATH_UNARY(double, log1p)
# endif
}
/* $$
The absolute value function is special because its $code std$$ name is
$code fabs$$
$codep */
namespace CppAD {
	inline double abs(const double& x)
	{	return std::fabs(x); }
}
/* $$

$head sign$$
The following defines the $code CppAD::sign$$ function that
is required to use $code AD<double>$$:
$codep */
namespace CppAD {
	inline double sign(const double& x)
	{	if( x > 0. )
			return 1.;
		if( x == 0. )
			return 0.;
		return -1.;
	}
}
/* $$

$head pow $$
The following defines a $code CppAD::pow$$ function that
is required to use $code AD<double>$$:
$codep */
namespace CppAD {
	inline double pow(const double& x, const double& y)
	{ return std::pow(x, y); }
}
/*$$

$head numeric_limits$$
The following defines the CppAD $cref numeric_limits$$
for the type $code double$$:
$codep */
namespace CppAD {
	CPPAD_NUMERIC_LIMITS(double, double)
}
/*$$

$head to_string$$
There is no need to define $code to_string$$ for $code double$$
because it is defined by including $code cppad/utility/to_string.hpp$$;
see $cref to_string$$.
See $cref/base_complex.hpp/base_complex.hpp/to_string/$$ for an example where
it is necessary to define $code to_string$$ for a $icode Base$$ type.

$end
*/

# endif
