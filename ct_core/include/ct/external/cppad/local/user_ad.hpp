// $Id: user_ad.hpp 3757 2015-11-30 12:03:07Z bradbell $
# ifndef CPPAD_USER_AD_HPP
# define CPPAD_USER_AD_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    Eclipse Public License Version 1.0.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */
/*
---------------------------------------------------------------------------

$begin AD$$
$spell
	std
	bool
	cos
	Cpp
$$

$section AD Objects$$
$mindex require$$


$head Purpose$$
The sections listed below describe the operations
that are available to $cref/AD of Base/glossary/AD of Base/$$ objects.
These objects are used to $cref/tape/glossary/Tape/$$
an AD of $icode Base$$
$cref/operation sequence/glossary/Operation/Sequence/$$.
This operation sequence can
be transferred to an $cref ADFun$$ object where it
can be used to evaluate the corresponding
function and derivative values.

$head Base Type Requirements$$
The $icode Base$$ requirements are provided by the CppAD package
for the following base types:
$code float$$,
$code double$$,
$code std::complex<float>$$,
$code std::complex<double>$$.
Otherwise, see $cref base_require$$.


$childtable%
	cppad/local/ad_ctor.hpp%
	cppad/local/ad_assign.hpp%
	cppad/local/convert.hpp%
	cppad/local/ad_valued.hpp%
	cppad/local/bool_valued.hpp%
	cppad/local/vec_ad.hpp%
	cppad/base_require.hpp
%$$

$end
---------------------------------------------------------------------------
*/

# include <cppad/local/ad_ctor.hpp>
# include <cppad/local/ad_assign.hpp>
# include <cppad/local/convert.hpp>
# include <cppad/local/vec_ad.hpp>
# include <cppad/local/ad_valued.hpp>
# include <cppad/local/bool_valued.hpp>
# include <cppad/local/zdouble.hpp>

# endif
