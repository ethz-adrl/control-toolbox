// $Id: add_op.hpp 3757 2015-11-30 12:03:07Z bradbell $
# ifndef CPPAD_ADD_OP_HPP
# define CPPAD_ADD_OP_HPP

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
\file add_op.hpp
Forward and reverse mode calculations for z = x + y.
*/

// --------------------------- Addvv -----------------------------------------
/*!
Compute forward mode Taylor coefficients for result of op = AddvvOp.

The C++ source code corresponding to this operation is
\verbatim
	z = x + y
\endverbatim
In the documentation below,
this operations is for the case where both x and y are variables
and the argument \a parameter is not used.

\copydetails forward_binary_op
*/

template <class Base>
inline void forward_addvv_op(
	size_t        p           ,
	size_t        q           ,
	size_t        i_z         ,
	const addr_t* arg         ,
	const Base*   parameter   ,
	size_t        cap_order   ,
	Base*         taylor      )
{
	// check assumptions
	CPPAD_ASSERT_UNKNOWN( NumArg(AddvvOp) == 2 );
	CPPAD_ASSERT_UNKNOWN( NumRes(AddvvOp) == 1 );
	CPPAD_ASSERT_UNKNOWN( q < cap_order );
	CPPAD_ASSERT_UNKNOWN( p <= q  );

	// Taylor coefficients corresponding to arguments and result
	Base* x = taylor + arg[0] * cap_order;
	Base* y = taylor + arg[1] * cap_order;
	Base* z = taylor + i_z    * cap_order;

	for(size_t j = p; j <= q; j++)
		z[j] = x[j] + y[j];
}
/*!
Multiple directions forward mode Taylor coefficients for op = AddvvOp.

The C++ source code corresponding to this operation is
\verbatim
	z = x + y
\endverbatim
In the documentation below,
this operations is for the case where both x and y are variables
and the argument \a parameter is not used.

\copydetails forward_binary_op_dir
*/

template <class Base>
inline void forward_addvv_op_dir(
	size_t        q           ,
	size_t        r           ,
	size_t        i_z         ,
	const addr_t* arg         ,
	const Base*   parameter   ,
	size_t        cap_order   ,
	Base*         taylor      )
{
	// check assumptions
	CPPAD_ASSERT_UNKNOWN( NumArg(AddvvOp) == 2 );
	CPPAD_ASSERT_UNKNOWN( NumRes(AddvvOp) == 1 );
	CPPAD_ASSERT_UNKNOWN( q < cap_order );
	CPPAD_ASSERT_UNKNOWN( 0 < q  );

	// Taylor coefficients corresponding to arguments and result
	size_t num_taylor_per_var = (cap_order-1) * r + 1;
	Base* x = taylor + arg[0] * num_taylor_per_var;
	Base* y = taylor + arg[1] * num_taylor_per_var;
	Base* z = taylor +    i_z * num_taylor_per_var;

	size_t m = (q-1)*r + 1 ;
	for(size_t ell = 0; ell < r; ell++)
		z[m+ell] = x[m+ell] + y[m+ell];
}

/*!
Compute zero order forward mode Taylor coefficients for result of op = AddvvOp.

The C++ source code corresponding to this operation is
\verbatim
	z = x + y
\endverbatim
In the documentation below,
this operations is for the case where both x and y are variables
and the argument \a parameter is not used.

\copydetails forward_binary_op_0
*/

template <class Base>
inline void forward_addvv_op_0(
	size_t        i_z         ,
	const addr_t* arg         ,
	const Base*   parameter   ,
	size_t        cap_order   ,
	Base*         taylor      )
{
	// check assumptions
	CPPAD_ASSERT_UNKNOWN( NumArg(AddvvOp) == 2 );
	CPPAD_ASSERT_UNKNOWN( NumRes(AddvvOp) == 1 );

	// Taylor coefficients corresponding to arguments and result
	Base* x = taylor + arg[0] * cap_order;
	Base* y = taylor + arg[1] * cap_order;
	Base* z = taylor + i_z    * cap_order;

	z[0] = x[0] + y[0];
}

/*!
Compute reverse mode partial derivatives for result of op = AddvvOp.

The C++ source code corresponding to this operation is
\verbatim
	z = x + y
\endverbatim
In the documentation below,
this operations is for the case where both x and y are variables
and the argument \a parameter is not used.

\copydetails reverse_binary_op
*/

template <class Base>
inline void reverse_addvv_op(
	size_t        d           ,
	size_t        i_z         ,
	const addr_t* arg         ,
	const Base*   parameter   ,
	size_t        cap_order   ,
	const Base*   taylor      ,
	size_t        nc_partial  ,
	Base*         partial     )
{
	// check assumptions
	CPPAD_ASSERT_UNKNOWN( NumArg(AddvvOp) == 2 );
	CPPAD_ASSERT_UNKNOWN( NumRes(AddvvOp) == 1 );
	CPPAD_ASSERT_UNKNOWN( d < cap_order );
	CPPAD_ASSERT_UNKNOWN( d < nc_partial );

	// Partial derivatives corresponding to arguments and result
	Base* px = partial + arg[0] * nc_partial;
	Base* py = partial + arg[1] * nc_partial;
	Base* pz = partial + i_z    * nc_partial;

	// number of indices to access
	size_t i = d + 1;
	while(i)
	{	--i;
		px[i] += pz[i];
		py[i] += pz[i];
	}
}

// --------------------------- Addpv -----------------------------------------
/*!
Compute forward mode Taylor coefficients for result of op = AddpvOp.

The C++ source code corresponding to this operation is
\verbatim
	z = x + y
\endverbatim
In the documentation below,
this operations is for the case where x is a parameter and y is a variable.

\copydetails forward_binary_op
*/
template <class Base>
inline void forward_addpv_op(
	size_t        p           ,
	size_t        q           ,
	size_t        i_z         ,
	const addr_t* arg         ,
	const Base*   parameter   ,
	size_t        cap_order   ,
	Base*         taylor      )
{
	// check assumptions
	CPPAD_ASSERT_UNKNOWN( NumArg(AddpvOp) == 2 );
	CPPAD_ASSERT_UNKNOWN( NumRes(AddpvOp) == 1 );
	CPPAD_ASSERT_UNKNOWN( q < cap_order );
	CPPAD_ASSERT_UNKNOWN( p <= q );

	// Taylor coefficients corresponding to arguments and result
	Base* y = taylor + arg[1] * cap_order;
	Base* z = taylor + i_z    * cap_order;

	if( p == 0 )
	{	// Paraemter value
		Base x = parameter[ arg[0] ];
		z[0] = x + y[0];
		p++;
	}
	for(size_t j = p; j <= q; j++)
		z[j] = y[j];
}
/*!
Multiple directions forward mode Taylor coefficients for op = AddpvOp.

The C++ source code corresponding to this operation is
\verbatim
	z = x + y
\endverbatim
In the documentation below,
this operations is for the case where x is a parameter and y is a variable.

\copydetails forward_binary_op_dir
*/
template <class Base>
inline void forward_addpv_op_dir(
	size_t        q           ,
	size_t        r           ,
	size_t        i_z         ,
	const addr_t* arg         ,
	const Base*   parameter   ,
	size_t        cap_order   ,
	Base*         taylor      )
{
	// check assumptions
	CPPAD_ASSERT_UNKNOWN( NumArg(AddpvOp) == 2 );
	CPPAD_ASSERT_UNKNOWN( NumRes(AddpvOp) == 1 );
	CPPAD_ASSERT_UNKNOWN( 0 < q );
	CPPAD_ASSERT_UNKNOWN( q < cap_order );

	// Taylor coefficients corresponding to arguments and result
	size_t num_taylor_per_var = (cap_order-1) * r + 1;
	size_t m                  = (q-1) * r + 1;
	Base* y = taylor + arg[1] * num_taylor_per_var + m;
	Base* z = taylor + i_z    * num_taylor_per_var + m;

	for(size_t ell = 0; ell < r; ell++)
		z[ell] = y[ell];
}
/*!
Compute zero order forward mode Taylor coefficient for result of op = AddpvOp.

The C++ source code corresponding to this operation is
\verbatim
	z = x + y
\endverbatim
In the documentation below,
this operations is for the case where x is a parameter and y is a variable.

\copydetails forward_binary_op_0
*/

template <class Base>
inline void forward_addpv_op_0(
	size_t        i_z         ,
	const addr_t* arg         ,
	const Base*   parameter   ,
	size_t        cap_order   ,
	Base*         taylor      )
{
	// check assumptions
	CPPAD_ASSERT_UNKNOWN( NumArg(AddpvOp) == 2 );
	CPPAD_ASSERT_UNKNOWN( NumRes(AddpvOp) == 1 );

	// Paraemter value
	Base x = parameter[ arg[0] ];

	// Taylor coefficients corresponding to arguments and result
	Base* y = taylor + arg[1] * cap_order;
	Base* z = taylor + i_z    * cap_order;

	z[0] = x + y[0];
}

/*!
Compute reverse mode partial derivative for result of op = AddpvOp.

The C++ source code corresponding to this operation is
\verbatim
	z = x + y
\endverbatim
In the documentation below,
this operations is for the case where x is a parameter and y is a variable.

\copydetails reverse_binary_op
*/

template <class Base>
inline void reverse_addpv_op(
	size_t        d           ,
	size_t        i_z         ,
	const addr_t* arg         ,
	const Base*   parameter   ,
	size_t        cap_order   ,
	const Base*   taylor      ,
	size_t        nc_partial  ,
	Base*         partial     )
{
	// check assumptions
	CPPAD_ASSERT_UNKNOWN( NumArg(AddvvOp) == 2 );
	CPPAD_ASSERT_UNKNOWN( NumRes(AddvvOp) == 1 );
	CPPAD_ASSERT_UNKNOWN( d < cap_order );
	CPPAD_ASSERT_UNKNOWN( d < nc_partial );

	// Partial derivatives corresponding to arguments and result
	Base* py = partial + arg[1] * nc_partial;
	Base* pz = partial + i_z    * nc_partial;

	// number of indices to access
	size_t i = d + 1;
	while(i)
	{	--i;
		py[i] += pz[i];
	}
}


} // END_CPPAD_NAMESPACE
# endif
