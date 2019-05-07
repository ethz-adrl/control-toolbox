// $Id: sparse_binary_op.hpp 3757 2015-11-30 12:03:07Z bradbell $
# ifndef CPPAD_SPARSE_BINARY_OP_HPP
# define CPPAD_SPARSE_BINARY_OP_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    Eclipse Public License Version 1.0.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file sparse_binary_op.hpp
Forward and reverse mode sparsity patterns for binary operators.
*/


/*!
Forward mode Jacobian sparsity pattern for all binary operators.

The C++ source code corresponding to a binary operation has the form
\verbatim
	z = fun(x, y)
\endverbatim
where fun is a C++ binary function and both x and y are variables,
or it has the form
\verbatim
	z = x op y
\endverbatim
where op is a C++ binary unary operator and both x and y are variables.

\tparam Vector_set
is the type used for vectors of sets. It can be either
\c sparse_pack, \c sparse_set, or \c sparse_list.

\param i_z
variable index corresponding to the result for this operation;
i.e., z.

\param arg
\a arg[0]
variable index corresponding to the left operand for this operator;
i.e., x.
\n
\n arg[1]
variable index corresponding to the right operand for this operator;
i.e., y.

\param sparsity
\b Input:
The set with index \a arg[0] in \a sparsity
is the sparsity bit pattern for x.
This identifies which of the independent variables the variable x
depends on.
\n
\n
\b Input:
The set with index \a arg[1] in \a sparsity
is the sparsity bit pattern for y.
This identifies which of the independent variables the variable y
depends on.
\n
\n
\b Output:
The set with index \a i_z in \a sparsity
is the sparsity bit pattern for z.
This identifies which of the independent variables the variable z
depends on.

\par Checked Assertions:
\li \a arg[0] < \a i_z
\li \a arg[1] < \a i_z
*/

template <class Vector_set>
inline void forward_sparse_jacobian_binary_op(
	size_t            i_z           ,
	const addr_t*     arg           ,
	Vector_set&       sparsity      )
{
	// check assumptions
	CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < i_z );
	CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < i_z );

	sparsity.binary_union(i_z, arg[0], arg[1], sparsity);

	return;
}

/*!
Reverse mode Jacobian sparsity pattern for all binary operators.

The C++ source code corresponding to a unary operation has the form
\verbatim
	z = fun(x, y)
\endverbatim
where fun is a C++ unary function and x and y are variables,
or it has the form
\verbatim
	z = x op y
\endverbatim
where op is a C++ bianry operator and x and y are variables.

This routine is given the sparsity patterns
for a function G(z, y, x, ... )
and it uses them to compute the sparsity patterns for
\verbatim
	H( y, x, w , u , ... ) = G[ z(x,y) , y , x , w , u , ... ]
\endverbatim

\tparam Vector_set
is the type used for vectors of sets. It can be either
\c sparse_pack, \c sparse_set, or \c sparse_list.

\param i_z
variable index corresponding to the result for this operation;
i.e., z.

\param arg
\a arg[0]
variable index corresponding to the left operand for this operator;
i.e., x.

\n
\n arg[1]
variable index corresponding to the right operand for this operator;
i.e., y.

\param sparsity
The set with index \a i_z in \a sparsity
is the sparsity pattern for z corresponding ot the function G.
\n
\n
The set with index \a arg[0] in \a sparsity
is the sparsity pattern for x.
On input, it corresponds to the function G,
and on output it corresponds to H.
\n
\n
The set with index \a arg[1] in \a sparsity
is the sparsity pattern for y.
On input, it corresponds to the function G,
and on output it corresponds to H.
\n
\n

\par Checked Assertions:
\li \a arg[0] < \a i_z
\li \a arg[1] < \a i_z
*/
template <class Vector_set>
inline void reverse_sparse_jacobian_binary_op(
	size_t              i_z           ,
	const addr_t*       arg           ,
	Vector_set&         sparsity      )
{
	// check assumptions
	CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < i_z );
	CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < i_z );

	sparsity.binary_union(arg[0], arg[0], i_z, sparsity);
	sparsity.binary_union(arg[1], arg[1], i_z, sparsity);

	return;
}

/*!
Reverse mode Hessian sparsity pattern for add and subtract operators.

The C++ source code corresponding to a unary operation has the form
\verbatim
	z = x op y
\endverbatim
where op is + or - and x, y are variables.

\copydetails reverse_sparse_hessian_binary_op
*/
template <class Vector_set>
inline void reverse_sparse_hessian_addsub_op(
	size_t               i_z                ,
	const addr_t*        arg                ,
	bool*                jac_reverse        ,
	Vector_set&          for_jac_sparsity   ,
	Vector_set&          rev_hes_sparsity   )
{
	// check assumptions
	CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < i_z );
	CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < i_z );

	rev_hes_sparsity.binary_union(arg[0], arg[0], i_z, rev_hes_sparsity);
	rev_hes_sparsity.binary_union(arg[1], arg[1], i_z, rev_hes_sparsity);

	jac_reverse[arg[0]] |= jac_reverse[i_z];
	jac_reverse[arg[1]] |= jac_reverse[i_z];

	return;
}

/*!
Reverse mode Hessian sparsity pattern for multiplication operator.

The C++ source code corresponding to a unary operation has the form
\verbatim
	z = x * y
\endverbatim
where x and y are variables.

\copydetails reverse_sparse_hessian_binary_op
*/
template <class Vector_set>
inline void reverse_sparse_hessian_mul_op(
	size_t               i_z                ,
	const addr_t*        arg                ,
	bool*                jac_reverse        ,
	Vector_set&          for_jac_sparsity   ,
	Vector_set&          rev_hes_sparsity   )
{
	// check assumptions
	CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < i_z );
	CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < i_z );

	rev_hes_sparsity.binary_union(arg[0], arg[0], i_z, rev_hes_sparsity);
	rev_hes_sparsity.binary_union(arg[1], arg[1], i_z, rev_hes_sparsity);

	if( jac_reverse[i_z] )
	{	rev_hes_sparsity.binary_union(
			arg[0], arg[0], arg[1], for_jac_sparsity);
		rev_hes_sparsity.binary_union(
			arg[1], arg[1], arg[0], for_jac_sparsity);
	}

	jac_reverse[arg[0]] |= jac_reverse[i_z];
	jac_reverse[arg[1]] |= jac_reverse[i_z];
	return;
}

/*!
Reverse mode Hessian sparsity pattern for division operator.

The C++ source code corresponding to a unary operation has the form
\verbatim
	z = x / y
\endverbatim
where x and y are variables.

\copydetails reverse_sparse_hessian_binary_op
*/
template <class Vector_set>
inline void reverse_sparse_hessian_div_op(
	size_t               i_z                ,
	const addr_t*        arg                ,
	bool*                jac_reverse        ,
	Vector_set&          for_jac_sparsity   ,
	Vector_set&          rev_hes_sparsity   )
{
	// check assumptions
	CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < i_z );
	CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < i_z );

	rev_hes_sparsity.binary_union(arg[0], arg[0], i_z, rev_hes_sparsity);
	rev_hes_sparsity.binary_union(arg[1], arg[1], i_z, rev_hes_sparsity);

	if( jac_reverse[i_z] )
	{	rev_hes_sparsity.binary_union(
			arg[0], arg[0], arg[1], for_jac_sparsity);
		rev_hes_sparsity.binary_union(
			arg[1], arg[1], arg[0], for_jac_sparsity);
		rev_hes_sparsity.binary_union(
			arg[1], arg[1], arg[1], for_jac_sparsity);
	}

	jac_reverse[arg[0]] |= jac_reverse[i_z];
	jac_reverse[arg[1]] |= jac_reverse[i_z];
	return;
}

/*!
Reverse mode Hessian sparsity pattern for power function.

The C++ source code corresponding to a unary operation has the form
\verbatim
	z = pow(x, y)
\endverbatim
where x and y are variables.

\copydetails reverse_sparse_hessian_binary_op
*/
template <class Vector_set>
inline void reverse_sparse_hessian_pow_op(
	size_t               i_z                ,
	const addr_t*        arg                ,
	bool*                jac_reverse        ,
	Vector_set&          for_jac_sparsity   ,
	Vector_set&          rev_hes_sparsity   )
{
	// check assumptions
	CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < i_z );
	CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < i_z );

	rev_hes_sparsity.binary_union(arg[0], arg[0], i_z, rev_hes_sparsity);
	rev_hes_sparsity.binary_union(arg[1], arg[1], i_z, rev_hes_sparsity);

	if( jac_reverse[i_z] )
	{
		rev_hes_sparsity.binary_union(
			arg[0], arg[0], arg[0], for_jac_sparsity);
		rev_hes_sparsity.binary_union(
			arg[0], arg[0], arg[1], for_jac_sparsity);

		rev_hes_sparsity.binary_union(
			arg[1], arg[1], arg[0], for_jac_sparsity);
		rev_hes_sparsity.binary_union(
			arg[1], arg[1], arg[1], for_jac_sparsity);
	}

	// I cannot think of a case where this is necessary, but it including
	// it makes it like the other cases.
	jac_reverse[arg[0]] |= jac_reverse[i_z];
	jac_reverse[arg[1]] |= jac_reverse[i_z];
	return;
}

} // END_CPPAD_NAMESPACE
# endif
