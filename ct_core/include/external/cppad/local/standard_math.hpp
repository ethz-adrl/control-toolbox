// $Id$
# ifndef CPPAD_STANDARD_MATH_HPP
# define CPPAD_STANDARD_MATH_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    Eclipse Public License Version 1.0.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

/*
$begin unary_standard_math$$
$spell
	const
	VecAD
	fabs
$$

$section The Unary Standard Math Functions$$

$head Syntax$$
$icode%y% = %fun%(%x%)%$$

$head Purpose$$
Evaluates the standard math function $icode fun$$.

$head Possible Types$$

$subhead Base$$
If $icode Base$$ satisfies the
$cref/base type requirements/base_require/$$
and argument $icode x$$ has prototype
$codei%
	const %Base%& %x%
%$$
then the result $icode y$$ has prototype
$codei%
	%Base% %y%
%$$

$subhead AD<Base>$$
If the argument $icode x$$ has prototype
$codei%
	const AD<%Base%>& %x%
%$$
then the result $icode y$$ has prototype
$codei%
	AD<%Base%> %y%
%$$

$subhead VecAD<Base>$$
If the argument $icode x$$ has prototype
$codei%
	const VecAD<%Base%>::reference& %x%
%$$
then the result $icode y$$ has prototype
$codei%
	AD<%Base%> %y%
%$$

$children%cppad/local/std_math_98.hpp
	%cppad/local/abs.hpp
	%cppad/local/acosh.hpp
	%cppad/local/asinh.hpp
	%cppad/local/atanh.hpp
	%cppad/local/erf.hpp
	%cppad/local/expm1.hpp
	%cppad/local/log1p.hpp
	%cppad/local/sign.hpp
%$$

$head fun$$
The possible values for $icode fun$$ are
$table
$icode  fun$$ $pre  $$ $cnext Description        $rnext
$cref abs$$            $cnext $title abs$$       $rnext
$cref acos$$           $cnext $title acos$$      $rnext
$cref acosh$$          $cnext $title acosh$$     $rnext
$cref asin$$           $cnext $title asin$$      $rnext
$cref asinh$$          $cnext $title asinh$$     $rnext
$cref atan$$           $cnext $title atan$$      $rnext
$cref atanh$$          $cnext $title atanh$$     $rnext
$cref cos$$            $cnext $title cos$$       $rnext
$cref cosh$$           $cnext $title cosh$$      $rnext
$cref erf$$            $cnext $title erf$$       $rnext
$cref exp$$            $cnext $title exp$$       $rnext
$cref expm1$$          $cnext $title expm1$$     $rnext
$cref/fabs/abs/$$      $cnext $title abs$$       $rnext
$cref log10$$          $cnext $title log10$$     $rnext
$cref log1p$$          $cnext $title log1p$$     $rnext
$cref log$$            $cnext $title log$$       $rnext
$cref sign$$           $cnext $title sign$$      $rnext
$cref sin$$            $cnext $title sin$$       $rnext
$cref sinh$$           $cnext $title sinh$$      $rnext
$cref sqrt$$           $cnext $title sqrt$$      $rnext
$cref tan$$            $cnext $title tan$$       $rnext
$cref tanh$$           $cnext $title tanh$$
$tend

$end
*/
# include <cppad/local/abs.hpp>
# include <cppad/local/acosh.hpp>
# include <cppad/local/asinh.hpp>
# include <cppad/local/atanh.hpp>
# include <cppad/local/erf.hpp>
# include <cppad/local/expm1.hpp>
# include <cppad/local/log1p.hpp>
# include <cppad/local/sign.hpp>
# include <cppad/local/sign.hpp>

/*
$begin binary_math$$

$section The Binary Math Functions$$

$childtable%cppad/local/atan2.hpp
	%cppad/local/pow.hpp
	%cppad/local/azmul.hpp
%$$

$end
*/
# include <cppad/local/atan2.hpp>
# include <cppad/local/pow.hpp>

# endif
