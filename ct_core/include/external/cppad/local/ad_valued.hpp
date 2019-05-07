// $Id: ad_valued.hpp 3757 2015-11-30 12:03:07Z bradbell $
# ifndef CPPAD_AD_VALUED_HPP
# define CPPAD_AD_VALUED_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    Eclipse Public License Version 1.0.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

/*
$begin ADValued$$
$spell
$$


$section AD Valued Operations and Functions$$

$comment atomic.omh includes atomic_base.omh which atomic_base.hpp$$
$childtable%
	cppad/local/arithmetic.hpp%
	cppad/local/standard_math.hpp%
	cppad/local/cond_exp.hpp%
	cppad/local/discrete.hpp%
	cppad/local/numeric_limits.hpp%
	omh/atomic.omh
%$$

$end
*/

// include MathOther.h after CondExp.h because some MathOther.h routines use
// CondExp.h and CondExp.h is not sufficently declared in Declare.h

# include <cppad/local/arithmetic.hpp>
# include <cppad/local/standard_math.hpp>
# include <cppad/local/azmul.hpp>
# include <cppad/local/cond_exp.hpp>
# include <cppad/local/discrete.hpp>
# include <cppad/local/atomic_base.hpp>
# include <cppad/local/checkpoint.hpp>
# include <cppad/local/old_atomic.hpp>

# endif
