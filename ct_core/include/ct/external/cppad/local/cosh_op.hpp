// $Id: cosh_op.hpp 3757 2015-11-30 12:03:07Z bradbell $
# ifndef CPPAD_COSH_OP_HPP
# define CPPAD_COSH_OP_HPP

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
\file cosh_op.hpp
Forward and reverse mode calculations for z = cosh(x).
*/


/*!
Compute forward mode Taylor coefficient for result of op = CoshOp.

The C++ source code corresponding to this operation is
\verbatim
	z = cosh(x)
\endverbatim
The auxillary result is
\verbatim
	y = sinh(x)
\endverbatim
The value of y, and its derivatives, are computed along with the value
and derivatives of z.

\copydetails forward_unary2_op
*/
template <class Base>
inline void forward_cosh_op(
	size_t p           ,
	size_t q           ,
	size_t i_z         ,
	size_t i_x         ,
	size_t cap_order   ,
	Base*  taylor      )
{
	// check assumptions
	CPPAD_ASSERT_UNKNOWN( NumArg(CoshOp) == 1 );
	CPPAD_ASSERT_UNKNOWN( NumRes(CoshOp) == 2 );
	CPPAD_ASSERT_UNKNOWN( q < cap_order );
	CPPAD_ASSERT_UNKNOWN( p <= q );

	// Taylor coefficients corresponding to argument and result
	Base* x = taylor + i_x * cap_order;
	Base* c = taylor + i_z * cap_order;
	Base* s = c      -       cap_order;

	// rest of this routine is identical for the following cases:
	// forward_sin_op, forward_cos_op, forward_sinh_op, forward_cosh_op.
	// (except that there is a sign difference for hyperbolic case).
	size_t k;
	if( p == 0 )
	{	s[0] = sinh( x[0] );
		c[0] = cosh( x[0] );
		p++;
	}
	for(size_t j = p; j <= q; j++)
	{
		s[j] = Base(0);
		c[j] = Base(0);
		for(k = 1; k <= j; k++)
		{	s[j] += Base(k) * x[k] * c[j-k];
			c[j] += Base(k) * x[k] * s[j-k];
		}
		s[j] /= Base(j);
		c[j] /= Base(j);
	}
}
/*!
Compute forward mode Taylor coefficient for result of op = CoshOp.

The C++ source code corresponding to this operation is
\verbatim
	z = cosh(x)
\endverbatim
The auxillary result is
\verbatim
	y = sinh(x)
\endverbatim
The value of y, and its derivatives, are computed along with the value
and derivatives of z.

\copydetails forward_unary2_op_dir
*/
template <class Base>
inline void forward_cosh_op_dir(
	size_t q           ,
	size_t r           ,
	size_t i_z         ,
	size_t i_x         ,
	size_t cap_order   ,
	Base*  taylor      )
{
	// check assumptions
	CPPAD_ASSERT_UNKNOWN( NumArg(CoshOp) == 1 );
	CPPAD_ASSERT_UNKNOWN( NumRes(CoshOp) == 2 );
	CPPAD_ASSERT_UNKNOWN( 0 < q );
	CPPAD_ASSERT_UNKNOWN( q < cap_order );

	// Taylor coefficients corresponding to argument and result
	size_t num_taylor_per_var = (cap_order-1) * r + 1;
	Base* x = taylor + i_x * num_taylor_per_var;
	Base* s = taylor + i_z * num_taylor_per_var;
	Base* c = s      -       num_taylor_per_var;


	// rest of this routine is identical for the following cases:
	// forward_sin_op, forward_cos_op, forward_sinh_op, forward_cosh_op
	// (except that there is a sign difference for the hyperbolic case).
	size_t m = (q-1) * r + 1;
	for(size_t ell = 0; ell < r; ell++)
	{	s[m+ell] = Base(q) * x[m + ell] * c[0];
		c[m+ell] = Base(q) * x[m + ell] * s[0];
		for(size_t k = 1; k < q; k++)
		{	s[m+ell] += Base(k) * x[(k-1)*r+1+ell] * c[(q-k-1)*r+1+ell];
			c[m+ell] += Base(k) * x[(k-1)*r+1+ell] * s[(q-k-1)*r+1+ell];
		}
		s[m+ell] /= Base(q);
		c[m+ell] /= Base(q);
	}
}

/*!
Compute zero order forward mode Taylor coefficient for result of op = CoshOp.

The C++ source code corresponding to this operation is
\verbatim
	z = cosh(x)
\endverbatim
The auxillary result is
\verbatim
	y = sinh(x)
\endverbatim
The value of y is computed along with the value of z.

\copydetails forward_unary2_op_0
*/
template <class Base>
inline void forward_cosh_op_0(
	size_t i_z         ,
	size_t i_x         ,
	size_t cap_order   ,
	Base*  taylor      )
{
	// check assumptions
	CPPAD_ASSERT_UNKNOWN( NumArg(CoshOp) == 1 );
	CPPAD_ASSERT_UNKNOWN( NumRes(CoshOp) == 2 );
	CPPAD_ASSERT_UNKNOWN( 0 < cap_order );

	// Taylor coefficients corresponding to argument and result
	Base* x = taylor + i_x * cap_order;
	Base* c = taylor + i_z * cap_order;  // called z in documentation
	Base* s = c      -       cap_order;  // called y in documentation

	c[0] = cosh( x[0] );
	s[0] = sinh( x[0] );
}
/*!
Compute reverse mode partial derivatives for result of op = CoshOp.

The C++ source code corresponding to this operation is
\verbatim
	z = cosh(x)
\endverbatim
The auxillary result is
\verbatim
	y = sinh(x)
\endverbatim
The value of y is computed along with the value of z.

\copydetails reverse_unary2_op
*/

template <class Base>
inline void reverse_cosh_op(
	size_t      d            ,
	size_t      i_z          ,
	size_t      i_x          ,
	size_t      cap_order    ,
	const Base* taylor       ,
	size_t      nc_partial   ,
	Base*       partial      )
{
	// check assumptions
	CPPAD_ASSERT_UNKNOWN( NumArg(CoshOp) == 1 );
	CPPAD_ASSERT_UNKNOWN( NumRes(CoshOp) == 2 );
	CPPAD_ASSERT_UNKNOWN( d < cap_order );
	CPPAD_ASSERT_UNKNOWN( d < nc_partial );

	// Taylor coefficients and partials corresponding to argument
	const Base* x  = taylor  + i_x * cap_order;
	Base* px       = partial + i_x * nc_partial;

	// Taylor coefficients and partials corresponding to first result
	const Base* c  = taylor  + i_z * cap_order; // called z in doc
	Base* pc       = partial + i_z * nc_partial;

	// Taylor coefficients and partials corresponding to auxillary result
	const Base* s  = c  - cap_order; // called y in documentation
	Base* ps       = pc - nc_partial;


	// rest of this routine is identical for the following cases:
	// reverse_sin_op, reverse_cos_op, reverse_sinh_op, reverse_cosh_op.
	size_t j = d;
	size_t k;
	while(j)
	{
		ps[j]   /= Base(j);
		pc[j]   /= Base(j);
		for(k = 1; k <= j; k++)
		{
			px[k]   += Base(k) * azmul(ps[j], c[j-k]);
			px[k]   += Base(k) * azmul(pc[j], s[j-k]);

			ps[j-k] += Base(k) * azmul(pc[j], x[k]);
			pc[j-k] += Base(k) * azmul(ps[j], x[k]);

		}
		--j;
	}
	px[0] += azmul(ps[0], c[0]);
	px[0] += azmul(pc[0], s[0]);
}

} // END_CPPAD_NAMESPACE
# endif
