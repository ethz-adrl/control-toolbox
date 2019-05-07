// $Id: base_float.hpp 3769 2015-12-29 16:13:16Z bradbell $
# ifndef CPPAD_BASE_FLOAT_HPP
# define CPPAD_BASE_FLOAT_HPP
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
$begin base_float.hpp$$
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


$section Enable use of AD<Base> where Base is float$$

$head CondExpOp$$
The type $code float$$ is a relatively simple type that supports
$code <$$, $code <=$$, $code ==$$, $code >=$$, and $code >$$ operators; see
$cref/ordered type/base_cond_exp/CondExpTemplate/Ordered Type/$$.
Hence its $code CondExpOp$$ function is defined by
$codep */
namespace CppAD {
	inline float CondExpOp(
		enum CompareOp     cop          ,
		const float&       left         ,
		const float&       right        ,
		const float&       exp_if_true  ,
		const float&       exp_if_false )
	{	return CondExpTemplate(cop, left, right, exp_if_true, exp_if_false);
	}
}
/* $$

$head CondExpRel$$
The $cref/CPPAD_COND_EXP_REL/base_cond_exp/CondExpRel/$$ macro invocation
$codep */
namespace CppAD {
	CPPAD_COND_EXP_REL(float)
}
/* $$
uses $code CondExpOp$$ above to
define $codei%CondExp%Rel%$$ for $code float$$ arguments
and $icode%Rel%$$ equal to
$code Lt$$, $code Le$$, $code Eq$$, $code Ge$$, and $code Gt$$.

$head EqualOpSeq$$
The type $code float$$ is simple (in this respect) and so we define
$codep */
namespace CppAD {
	inline bool EqualOpSeq(const float& x, const float& y)
	{	return x == y; }
}
/* $$

$head Identical$$
The type $code float$$ is simple (in this respect) and so we define
$codep */
namespace CppAD {
	inline bool IdenticalPar(const float& x)
	{	return true; }
	inline bool IdenticalZero(const float& x)
	{	return (x == 0.f); }
	inline bool IdenticalOne(const float& x)
	{	return (x == 1.f); }
	inline bool IdenticalEqualPar(const float& x, const float& y)
	{	return (x == y); }
}
/* $$

$head Integer$$
$codep */
namespace CppAD {
	inline int Integer(const float& x)
	{	return static_cast<int>(x); }
}
/* $$

$head azmul$$
$codep */
namespace CppAD {
	CPPAD_AZMUL( float )
}
/* $$

$head Ordered$$
The $code float$$ type supports ordered comparisons
$codep */
namespace CppAD {
	inline bool GreaterThanZero(const float& x)
	{	return x > 0.f; }
	inline bool GreaterThanOrZero(const float& x)
	{	return x >= 0.f; }
	inline bool LessThanZero(const float& x)
	{	return x < 0.f; }
	inline bool LessThanOrZero(const float& x)
	{	return x <= 0.f; }
	inline bool abs_geq(const float& x, const float& y)
	{	return std::fabs(x) >= std::fabs(y); }
}
/* $$

$head Unary Standard Math$$
The following macro invocations define the unary standard math functions
required to use $code AD<float>$$:
(in the CppAD namespace)
$codep */
namespace CppAD {
	CPPAD_STANDARD_MATH_UNARY(float, acos)
	CPPAD_STANDARD_MATH_UNARY(float, asin)
	CPPAD_STANDARD_MATH_UNARY(float, atan)
	CPPAD_STANDARD_MATH_UNARY(float, cos)
	CPPAD_STANDARD_MATH_UNARY(float, cosh)
	CPPAD_STANDARD_MATH_UNARY(float, exp)
	CPPAD_STANDARD_MATH_UNARY(float, fabs)
	CPPAD_STANDARD_MATH_UNARY(float, log)
	CPPAD_STANDARD_MATH_UNARY(float, log10)
	CPPAD_STANDARD_MATH_UNARY(float, sin)
	CPPAD_STANDARD_MATH_UNARY(float, sinh)
	CPPAD_STANDARD_MATH_UNARY(float, sqrt)
	CPPAD_STANDARD_MATH_UNARY(float, tan)
	CPPAD_STANDARD_MATH_UNARY(float, tanh)
# if CPPAD_USE_CPLUSPLUS_2011
	CPPAD_STANDARD_MATH_UNARY(float, erf)
	CPPAD_STANDARD_MATH_UNARY(float, asinh)
	CPPAD_STANDARD_MATH_UNARY(float, acosh)
	CPPAD_STANDARD_MATH_UNARY(float, atanh)
	CPPAD_STANDARD_MATH_UNARY(float, expm1)
	CPPAD_STANDARD_MATH_UNARY(float, log1p)
# endif
}
/* $$
The absolute value function is special because its $code std$$ name is
$code fabs$$
$codep */
namespace CppAD {
	inline float abs(const float& x)
	{	return std::fabs(x); }
}
/* $$

$head sign$$
The following defines the $code CppAD::sign$$ function that
is required to use $code AD<float>$$:
$codep */
namespace CppAD {
	inline float sign(const float& x)
	{	if( x > 0.f )
			return 1.f;
		if( x == 0.f )
			return 0.f;
		return -1.f;
	}
}
/* $$

$head pow $$
The following defines a $code CppAD::pow$$ function that
is required to use $code AD<float>$$:
$codep */
namespace CppAD {
	inline float pow(const float& x, const float& y)
	{ return std::pow(x, y); }
}
/*$$

$head numeric_limits$$
The following defines the CppAD $cref numeric_limits$$
for the type $code float$$:
$codep */
namespace CppAD {
	CPPAD_NUMERIC_LIMITS(float, float)
}
/*$$

$head to_string$$
There is no need to define $code to_string$$ for $code float$$
because it is defined by including $code cppad/utility/to_string.hpp$$;
see $cref to_string$$.
See $cref/base_complex.hpp/base_complex.hpp/to_string/$$ for an example where
it is necessary to define $code to_string$$ for a $icode Base$$ type.

$end
*/


# endif
