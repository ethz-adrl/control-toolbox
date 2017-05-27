// $Id: sign.hpp 3757 2015-11-30 12:03:07Z bradbell $
# ifndef CPPAD_SIGN_HPP
# define CPPAD_SIGN_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    Eclipse Public License Version 1.0.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

/*
$begin sign$$
$spell
	CppAD
	Dirac
$$
$section The Sign: sign$$

$head Syntax$$
$icode%y% = sign(%x%)%$$

$head Description$$
Evaluates the $code sign$$ function which is defined by
$latex \[
{\rm sign} (x) =
\left\{ \begin{array}{rl}
	+1 & {\rm if} \; x > 0 \\
	0  & {\rm if} \; x = 0 \\
	-1 & {\rm if} \; x < 0
\end{array} \right.
\] $$

$head x, y$$
See the $cref/possible types/unary_standard_math/Possible Types/$$
for a unary standard math function.

$head Atomic$$
This is an $cref/atomic operation/glossary/Operation/Atomic/$$.

$head Derivative$$
CppAD computes the derivative of the $code sign$$ function as zero for all
argument values $icode x$$.
The correct mathematical derivative is different and
is given by
$latex \[
	{\rm sign}^{(1)} (x) =  2 \delta (x)
\] $$
where $latex \delta (x)$$ is the Dirac Delta function.

$head Example$$
$children%
	example/sign.cpp
%$$
The file
$cref sign.cpp$$
contains an example and test of this function.
It returns true if it succeeds and false otherwise.

$end
-------------------------------------------------------------------------------
*/

//  BEGIN CppAD namespace
namespace CppAD {

template <class Base>
AD<Base> AD<Base>::Sign (void) const
{
	AD<Base> result;
	result.value_ = sign(value_);
	CPPAD_ASSERT_UNKNOWN( Parameter(result) );

	if( Variable(*this) )
	{	// add this operation to the tape
		CPPAD_ASSERT_UNKNOWN( NumRes(SignOp) == 1 );
		CPPAD_ASSERT_UNKNOWN( NumArg(SignOp) == 1 );
		ADTape<Base> *tape = tape_this();

		// corresponding operand address
		tape->Rec_.PutArg(taddr_);
		// put operator in the tape
		result.taddr_ = tape->Rec_.PutOp(SignOp);
		// make result a variable
		result.tape_id_    = tape->id_;
	}
	return result;
}

template <class Base>
inline AD<Base> sign(const AD<Base> &x)
{	return x.Sign(); }

template <class Base>
inline AD<Base> sign(const VecAD_reference<Base> &x)
{	return sign( x.ADBase() ); }

} // END CppAD namespace

# endif
