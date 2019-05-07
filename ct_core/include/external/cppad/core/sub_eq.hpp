// $Id$
# ifndef CPPAD_CORE_SUB_EQ_HPP
# define CPPAD_CORE_SUB_EQ_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    Eclipse Public License Version 1.0.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

//  BEGIN CppAD namespace
namespace CppAD {

template <class Base>
AD<Base>& AD<Base>::operator -= (const AD<Base> &right)
{
	// compute the Base part
	Base left;
	left    = value_;
	value_ -= right.value_;

	// check if there is a recording in progress
	local::ADTape<Base>* tape = AD<Base>::tape_ptr();
	if( tape == CPPAD_NULL )
		return *this;
	tape_id_t tape_id = tape->id_;

	// tape_id cannot match the default value for tape_id_; i.e., 0
	CPPAD_ASSERT_UNKNOWN( tape_id > 0 );
	bool var_left  = tape_id_       == tape_id;
	bool var_right = right.tape_id_ == tape_id;

	if( var_left )
	{	if( var_right )
		{	// this = variable - variable
			CPPAD_ASSERT_UNKNOWN( local::NumRes(local::SubvvOp) == 1 );
			CPPAD_ASSERT_UNKNOWN( local::NumArg(local::SubvvOp) == 2 );

			// put operand addresses in tape
			tape->Rec_.PutArg(taddr_, right.taddr_);
			// put operator in the tape
			taddr_ = tape->Rec_.PutOp(local::SubvvOp);
			// make this a variable
			CPPAD_ASSERT_UNKNOWN( tape_id_ == tape_id );
		}
		else if( IdenticalZero( right.value_ ) )
		{	// this = variable - 0
		}
		else
		{	// this = variable - parameter
			CPPAD_ASSERT_UNKNOWN( local::NumRes(local::SubvpOp) == 1 );
			CPPAD_ASSERT_UNKNOWN( local::NumArg(local::SubvpOp) == 2 );

			// put operand addresses in tape
			addr_t p = tape->Rec_.PutPar(right.value_);
			tape->Rec_.PutArg(taddr_, p);
			// put operator in the tape
			taddr_ = tape->Rec_.PutOp(local::SubvpOp);
			// make this a variable
			CPPAD_ASSERT_UNKNOWN( tape_id_ == tape_id );
		}
	}
	else if( var_right  )
	{	// this = parameter - variable
		CPPAD_ASSERT_UNKNOWN( local::NumRes(local::SubpvOp) == 1 );
		CPPAD_ASSERT_UNKNOWN( local::NumArg(local::SubpvOp) == 2 );

		// put operand addresses in tape
		addr_t p = tape->Rec_.PutPar(left);
		tape->Rec_.PutArg(p, right.taddr_);
		// put operator in the tape
		taddr_ = tape->Rec_.PutOp(local::SubpvOp);
		// make this a variable
		tape_id_ = tape_id;
	}
	return *this;
}

CPPAD_FOLD_ASSIGNMENT_OPERATOR(-=)

} // END CppAD namespace

# endif
