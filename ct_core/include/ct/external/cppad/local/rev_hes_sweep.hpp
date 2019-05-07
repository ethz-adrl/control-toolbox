// $Id: rev_hes_sweep.hpp 3757 2015-11-30 12:03:07Z bradbell $
# ifndef CPPAD_REV_HES_SWEEP_HPP
# define CPPAD_REV_HES_SWEEP_HPP

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
\file rev_hes_sweep.hpp
Compute Reverse mode Hessian sparsity patterns.
*/

/*!
\def CPPAD_REV_HES_SWEEP_TRACE
This value is either zero or one.
Zero is the normal operational value.
If it is one, a trace of every rev_hes_sweep computation is printed.
*/
# define CPPAD_REV_HES_SWEEP_TRACE 0

/*!
Given the forward Jacobian sparsity pattern for all the variables,
and the reverse Jacobian sparsity pattern for the dependent variables,
RevHesSweep computes the Hessian sparsity pattern for all the independent
variables.

\tparam Base
base type for the operator; i.e., this operation sequence was recorded
using AD< \a Base > and computations by this routine are done using type
\a Base.

\tparam Vector_set
is the type used for vectors of sets. It can be either
\c sparse_pack, \c sparse_set, or \c sparse_list.

\param n
is the number of independent variables on the tape.

\param numvar
is the total number of variables on the tape; i.e.,
\a play->num_var_rec().
This is also the number of rows in the entire sparsity pattern
\a rev_hes_sparse.

\param play
The information stored in \a play
is a recording of the operations corresponding to a function
\f[
	F : {\bf R}^n \rightarrow {\bf R}^m
\f]
where \f$ n \f$ is the number of independent variables
and \f$ m \f$ is the number of dependent variables.
The object \a play is effectly constant.
It is not declared const because while playing back the tape
the object \a play holds information about the currentl location
with in the tape and this changes during playback.

\param for_jac_sparse
For i = 0 , ... , \a numvar - 1,
(for all the variables on the tape),
the forward Jacobian sparsity pattern for the variable with index i
corresponds to the set with index i in \a for_jac_sparse.

\param RevJac
\b Input:
For i = 0, ... , \a numvar - 1
the if the variable with index i on the tape is an dependent variable and
included in the Hessian, \a RevJac[ i ] is equal to true,
otherwise it is equal to false.
\n
\n
\b Output: The values in \a RevJac upon return are not specified; i.e.,
it is used for temporary work space.

\param rev_hes_sparse
The reverse Hessian sparsity pattern for the variable with index i
corresponds to the set with index i in \a rev_hes_sparse.
\n
\n
\b Input: For i = 0 , ... , \a numvar - 1
the reverse Hessian sparsity pattern for the variable with index i is empty.
\n
\n
\b Output: For j = 1 , ... , \a n,
the reverse Hessian sparsity pattern for the independent dependent variable
with index (j-1) is given by the set with index j
in \a rev_hes_sparse.
The values in the rest of \a rev_hes_sparse are not specified; i.e.,
they are used for temporary work space.
*/

template <class Base, class Vector_set>
void RevHesSweep(
	size_t                n,
	size_t                numvar,
	player<Base>         *play,
	Vector_set&           for_jac_sparse, // should be const
	bool*                 RevJac,
	Vector_set&           rev_hes_sparse
)
{
	OpCode           op;
	size_t         i_op;
	size_t        i_var;

	const addr_t*   arg = CPPAD_NULL;

	// length of the parameter vector (used by CppAD assert macros)
	const size_t num_par = play->num_par_rec();

	size_t             i, j, k;

	// check numvar argument
	CPPAD_ASSERT_UNKNOWN( play->num_var_rec()     == numvar );
	CPPAD_ASSERT_UNKNOWN( for_jac_sparse.n_set() == numvar );
	CPPAD_ASSERT_UNKNOWN( rev_hes_sparse.n_set() == numvar );
	CPPAD_ASSERT_UNKNOWN( numvar > 0 );

	// upper limit exclusive for set elements
	size_t limit   = rev_hes_sparse.end();
	CPPAD_ASSERT_UNKNOWN( for_jac_sparse.end() == limit );

	// check number of sets match
	CPPAD_ASSERT_UNKNOWN(
		for_jac_sparse.n_set() == rev_hes_sparse.n_set()
	);

	// vecad_sparsity contains a sparsity pattern for each VecAD object.
	// vecad_ind maps a VecAD index (beginning of the VecAD object)
	// to the index for the corresponding set in vecad_sparsity.
	size_t num_vecad_ind   = play->num_vec_ind_rec();
	size_t num_vecad_vec   = play->num_vecad_vec_rec();
	Vector_set vecad_sparse;
	vecad_sparse.resize(num_vecad_vec, limit);
	pod_vector<size_t> vecad_ind;
	pod_vector<bool>   vecad_jac;
	if( num_vecad_vec > 0 )
	{	size_t length;
		vecad_ind.extend(num_vecad_ind);
		vecad_jac.extend(num_vecad_vec);
		j             = 0;
		for(i = 0; i < num_vecad_vec; i++)
		{	// length of this VecAD
			length   = play->GetVecInd(j);
			// set vecad_ind to proper index for this VecAD
			vecad_ind[j] = i;
			// make all other values for this vector invalid
			for(k = 1; k <= length; k++)
				vecad_ind[j+k] = num_vecad_vec;
			// start of next VecAD
			j       += length + 1;
			// initialize this vector's reverse jacobian value
			vecad_jac[i] = false;
		}
		CPPAD_ASSERT_UNKNOWN( j == play->num_vec_ind_rec() );
	}

	// work space used by UserOp.
	vector<size_t>     user_ix;  // variable indices for argument vector x
	typedef std::set<size_t> size_set;
	size_set::iterator set_itr;  // iterator for a standard set
	size_set::iterator set_end;  // end of iterator sequence
	vector< size_set > set_r;    // forward Jacobian sparsity for x
	vector< size_set > set_u;    // reverse Hessian sparsity for y
	vector< size_set > set_v;    // reverse Hessian sparsity for x
	//
	vector<bool>       bool_r;   // bool forward Jacobian sparsity for x
	vector<bool>       bool_u;   // bool reverse Hessian sparsity for y
	vector<bool>       bool_v;   // bool reverse Hessian sparsity for x
	//
	vectorBool         pack_r;   // pack forward Jacobian sparsity for x
	vectorBool         pack_u;   // pack reverse Hessian sparsity for y
	vectorBool         pack_v;   // pack reverse Hessian sparsity for x
	//
	vector<bool>       user_vx;  // which components of x are variables
	vector<bool>       user_s;   // reverse Jacobian sparsity for y
	vector<bool>       user_t;   // reverse Jacobian sparsity for x
	const size_t user_q = limit; // maximum element plus one
	size_t user_index = 0;       // indentifier for this atomic operation
	size_t user_id    = 0;       // user identifier for this call to operator
	size_t user_i     = 0;       // index in result vector
	size_t user_j     = 0;       // index in argument vector
	size_t user_m     = 0;       // size of result vector
	size_t user_n     = 0;       // size of arugment vector
	//
	atomic_base<Base>* user_atom = CPPAD_NULL; // user's atomic op calculator
	bool               user_pack = false;      // sparsity pattern type is pack
	bool               user_bool = false;      // sparsity pattern type is bool
	bool               user_set  = false;      // sparsity pattern type is set
# ifndef NDEBUG
	bool               user_ok   = false;      // atomic op return value
# endif
	// next expected operator in a UserOp sequence
	enum { user_start, user_arg, user_ret, user_end } user_state = user_end;


	// Initialize
	play->reverse_start(op, arg, i_op, i_var);
	CPPAD_ASSERT_UNKNOWN( op == EndOp );
# if CPPAD_REV_HES_SWEEP_TRACE
	std::cout << std::endl;
	CppAD::vectorBool zf_value(limit);
	CppAD::vectorBool zh_value(limit);
# endif
	bool more_operators = true;
	while(more_operators)
	{
		// next op
		play->reverse_next(op, arg, i_op, i_var);
# ifndef NDEBUG
		if( i_op <= n )
		{	CPPAD_ASSERT_UNKNOWN((op == InvOp) | (op == BeginOp));
		}
		else	CPPAD_ASSERT_UNKNOWN((op != InvOp) & (op != BeginOp));
# endif

		// rest of information depends on the case
		switch( op )
		{
			case AbsOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1)
			reverse_sparse_hessian_linear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case AddvvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1)
			reverse_sparse_hessian_addsub_op(
			i_var, arg, RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case AddpvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1)
			reverse_sparse_hessian_linear_unary_op(
			i_var, arg[1], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case AcosOp:
			// sqrt(1 - x * x), acos(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

# if CPPAD_USE_CPLUSPLUS_2011
			case AcoshOp:
			// sqrt(x * x - 1), acosh(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
# endif
			// -------------------------------------------------

			case AsinOp:
			// sqrt(1 - x * x), asin(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

# if CPPAD_USE_CPLUSPLUS_2011
			case AsinhOp:
			// sqrt(1 + x * x), asinh(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
# endif
			// -------------------------------------------------

			case AtanOp:
			// 1 + x * x, atan(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

# if CPPAD_USE_CPLUSPLUS_2011
			case AtanhOp:
			// 1 - x * x, atanh(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
# endif
			// -------------------------------------------------

			case BeginOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1)
			more_operators = false;
			break;
			// -------------------------------------------------

			case CSkipOp:
			// CSkipOp has a variable number of arguments and
			// reverse_next thinks it one has one argument.
			// We must inform reverse_next of this special case.
			play->reverse_cskip(op, arg, i_op, i_var);
			break;
			// -------------------------------------------------

			case CSumOp:
			// CSumOp has a variable number of arguments and
			// reverse_next thinks it one has one argument.
			// We must inform reverse_next of this special case.
			play->reverse_csum(op, arg, i_op, i_var);
			reverse_sparse_hessian_csum_op(
				i_var, arg, RevJac, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case CExpOp:
			reverse_sparse_hessian_cond_op(
				i_var, arg, num_par, RevJac, rev_hes_sparse
			);
			break;
			// ---------------------------------------------------

			case CosOp:
			// sin(x), cos(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// ---------------------------------------------------

			case CoshOp:
			// sinh(x), cosh(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case DisOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1)
			// derivativve is identically zero
			break;
			// -------------------------------------------------

			case DivvvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1)
			reverse_sparse_hessian_div_op(
			i_var, arg, RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case DivpvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[1], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case DivvpOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1)
			reverse_sparse_hessian_linear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case ErfOp:
			// arg[1] is always the parameter 0
			// arg[0] is always the parameter 2 / sqrt(pi)
			CPPAD_ASSERT_NARG_NRES(op, 3, 5);
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case ExpOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

# if CPPAD_USE_CPLUSPLUS_2011
			case Expm1Op:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
# endif
			// -------------------------------------------------

			case InvOp:
			CPPAD_ASSERT_NARG_NRES(op, 0, 1)
			// Z is already defined
			break;
			// -------------------------------------------------

			case LdpOp:
			reverse_sparse_hessian_load_op(
				op,
				i_var,
				arg,
				num_vecad_ind,
				vecad_ind.data(),
				rev_hes_sparse,
				vecad_sparse,
				RevJac,
				vecad_jac.data()
			);
			break;
			// -------------------------------------------------

			case LdvOp:
			reverse_sparse_hessian_load_op(
				op,
				i_var,
				arg,
				num_vecad_ind,
				vecad_ind.data(),
				rev_hes_sparse,
				vecad_sparse,
				RevJac,
				vecad_jac.data()
			);
			break;
			// -------------------------------------------------

			case EqpvOp:
			case EqvvOp:
			case LtpvOp:
			case LtvpOp:
			case LtvvOp:
			case LepvOp:
			case LevpOp:
			case LevvOp:
			case NepvOp:
			case NevvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 0);
			break;
			// -------------------------------------------------

			case LogOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

# if CPPAD_USE_CPLUSPLUS_2011
			case Log1pOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
# endif
			// -------------------------------------------------

			case MulpvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1)
			reverse_sparse_hessian_linear_unary_op(
			i_var, arg[1], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case MulvvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1)
			reverse_sparse_hessian_mul_op(
			i_var, arg, RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case ParOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1)

			break;
			// -------------------------------------------------

			case PowpvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 3)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[1], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case PowvpOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 3)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case PowvvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 3)
                        reverse_sparse_hessian_pow_op(
			i_var, arg, RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case PriOp:
			CPPAD_ASSERT_NARG_NRES(op, 5, 0);
			break;
			// -------------------------------------------------

			case SignOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1);
			// Derivative is identiaclly zero
			break;
			// -------------------------------------------------

			case SinOp:
			// cos(x), sin(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case SinhOp:
			// cosh(x), sinh(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case SqrtOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case StppOp:
			// sparsity cannot propagate through a parameter
			CPPAD_ASSERT_NARG_NRES(op, 3, 0)
			break;
			// -------------------------------------------------

			case StpvOp:
			reverse_sparse_hessian_store_op(
				op,
				arg,
				num_vecad_ind,
				vecad_ind.data(),
				rev_hes_sparse,
				vecad_sparse,
				RevJac,
				vecad_jac.data()
			);
			break;
			// -------------------------------------------------

			case StvpOp:
			// sparsity cannot propagate through a parameter
			CPPAD_ASSERT_NARG_NRES(op, 3, 0)
			break;
			// -------------------------------------------------

			case StvvOp:
			reverse_sparse_hessian_store_op(
				op,
				arg,
				num_vecad_ind,
				vecad_ind.data(),
				rev_hes_sparse,
				vecad_sparse,
				RevJac,
				vecad_jac.data()
			);
			break;
			// -------------------------------------------------

			case SubvvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1)
			reverse_sparse_hessian_addsub_op(
			i_var, arg, RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case SubpvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1)
			reverse_sparse_hessian_linear_unary_op(
			i_var, arg[1], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case SubvpOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1)
			reverse_sparse_hessian_linear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case TanOp:
			// tan(x)^2, tan(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case TanhOp:
			// tanh(x)^2, tanh(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2)
			reverse_sparse_hessian_nonlinear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case UserOp:
			// start or end an atomic operation sequence
			CPPAD_ASSERT_UNKNOWN( NumRes( UserOp ) == 0 );
			CPPAD_ASSERT_UNKNOWN( NumArg( UserOp ) == 4 );
			if( user_state == user_end )
			{	user_index = arg[0];
				user_id    = arg[1];
				user_n     = arg[2];
				user_m     = arg[3];
				user_atom  = atomic_base<Base>::class_object(user_index);
# ifndef NDEBUG
				if( user_atom == CPPAD_NULL )
				{	std::string msg =
						atomic_base<Base>::class_name(user_index)
						+ ": atomic_base function has been deleted";
					CPPAD_ASSERT_KNOWN(false, msg.c_str() );
				}
# endif
				user_pack  = user_atom->sparsity() ==
							atomic_base<Base>::pack_sparsity_enum;
				user_bool  = user_atom->sparsity() ==
							atomic_base<Base>::bool_sparsity_enum;
				user_set   = user_atom->sparsity() ==
							atomic_base<Base>::set_sparsity_enum;
				CPPAD_ASSERT_UNKNOWN( user_pack || user_bool || user_set );
				user_ix.resize(user_n);
				user_vx.resize(user_n);
				user_s.resize(user_m);
				user_t.resize(user_n);

				// simpler to initialize all sparsity patterns as empty
				for(i = 0; i < user_m; i++)
					user_s[i] = false;
				for(i = 0; i < user_n; i++)
					user_t[i] = false;
				if( user_pack )
				{	pack_r.resize(user_n * user_q);
					pack_u.resize(user_m * user_q);
					pack_v.resize(user_n * user_q);
					// simpler to initialize all patterns as empty
					for(i = 0; i < user_m; i++)
					{
						for(j = 0; j < user_q; j++)
							pack_u[ i * user_q + j] = false;
					}
					for(i = 0; i < user_n; i++)
					{
						for(j = 0; j < user_q; j++)
						{	pack_r[ i * user_q + j] = false;
							pack_v[ i * user_q + j] = false;
						}
					}
				}
				if( user_bool )
				{	bool_r.resize(user_n * user_q);
					bool_u.resize(user_m * user_q);
					bool_v.resize(user_n * user_q);
					// simpler to initialize all patterns as empty
					for(i = 0; i < user_m; i++)
					{
						for(j = 0; j < user_q; j++)
							bool_u[ i * user_q + j] = false;
					}
					for(i = 0; i < user_n; i++)
					{
						for(j = 0; j < user_q; j++)
						{	bool_r[ i * user_q + j] = false;
							bool_v[ i * user_q + j] = false;
						}
					}
				}
				if( user_set )
				{	set_r.resize(user_n);
					set_u.resize(user_m);
					set_v.resize(user_n);
					for(i = 0; i < user_m; i++)
						set_u[i].clear();
					for(i = 0; i < user_n; i++)
					{	set_r[i].clear();
						set_v[i].clear();
					}
				}
				user_j     = user_n;
				user_i     = user_m;
				user_state = user_ret;
			}
			else
			{	CPPAD_ASSERT_UNKNOWN( user_state == user_start );
				CPPAD_ASSERT_UNKNOWN( user_index == size_t(arg[0]) );
				CPPAD_ASSERT_UNKNOWN( user_id    == size_t(arg[1]) );
				CPPAD_ASSERT_UNKNOWN( user_n     == size_t(arg[2]) );
				CPPAD_ASSERT_UNKNOWN( user_m     == size_t(arg[3]) );
				user_state = user_end;

				// call users function for this operation
				user_atom->set_id(user_id);
# ifdef NDEBUG
				if( user_pack )
					user_atom->rev_sparse_hes(user_vx,
						user_s, user_t, user_q, pack_r, pack_u, pack_v
				);
				if( user_bool )
					user_atom->rev_sparse_hes(user_vx,
						user_s, user_t, user_q, bool_r, bool_u, bool_v
				);
				if( user_set )
					user_atom->rev_sparse_hes(user_vx,
						user_s, user_t, user_q, set_r, set_u, set_v
				);
# else
				if( user_pack )
					user_ok = user_atom->rev_sparse_hes(user_vx,
						user_s, user_t, user_q, pack_r, pack_u, pack_v
				);
				if( user_bool )
					user_ok = user_atom->rev_sparse_hes(user_vx,
						user_s, user_t, user_q, bool_r, bool_u, bool_v
				);
				if( user_set )
					user_ok = user_atom->rev_sparse_hes(user_vx,
						user_s, user_t, user_q, set_r, set_u, set_v
				);
				if( ! user_ok )
				{	std::string msg =
						atomic_base<Base>::class_name(user_index)
						+ ": atomic_base.rev_sparse_hes: returned false\n";
					if( user_pack )
						msg += "sparsity = pack_sparsity_enum";
					if( user_bool )
						msg += "sparsity = bool_sparsity_enum";
					if( user_set )
						msg += "sparsity = set_sparsity_enum";
					CPPAD_ASSERT_KNOWN(false, msg.c_str() );
				}
# endif
				for(i = 0; i < user_n; i++) if( user_ix[i] > 0 )
				{
					size_t  i_x = user_ix[i];
					if( user_t[i] )
						RevJac[i_x] = true;
					if( user_pack )
					{
						for(j = 0; j < user_q; j++)
							if( pack_v[ i * user_q + j ] )
								rev_hes_sparse.add_element(i_x, j);
					}
					if( user_bool )
					{
						for(j = 0; j < user_q; j++)
							if( bool_v[ i * user_q + j ] )
								rev_hes_sparse.add_element(i_x, j);
					}
					if( user_set )
					{
						set_itr = set_v[i].begin();
						set_end = set_v[i].end();
						while( set_itr != set_end )
							rev_hes_sparse.add_element(i_x, *set_itr++);
					}
				}
               }
			break;

			case UsrapOp:
			// parameter argument in an atomic operation sequence
			CPPAD_ASSERT_UNKNOWN( user_state == user_arg );
			CPPAD_ASSERT_UNKNOWN( 0 < user_j && user_j <= user_n );
			CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 );
			CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_par );
			--user_j;
			user_ix[user_j] = 0;
			user_vx[user_j] = false;
			if( user_j == 0 )
				user_state = user_start;
			break;

			case UsravOp:
			// variable argument in an atomic operation sequence
			CPPAD_ASSERT_UNKNOWN( user_state == user_arg );
			CPPAD_ASSERT_UNKNOWN( 0 < user_j && user_j <= user_n );
			CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 );
			CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) <= i_var );
			CPPAD_ASSERT_UNKNOWN( 0 < arg[0] );
			--user_j;
			user_ix[user_j] = arg[0];
			user_vx[user_j] = true;
			for_jac_sparse.begin(arg[0]);
			i = for_jac_sparse.next_element();
			while( i < user_q )
			{	if( user_pack )
					pack_r[ user_j * user_q + i ] = true;
				if( user_bool )
					bool_r[ user_j * user_q + i ] = true;
				if( user_set )
					set_r[user_j].insert(i);
				i = for_jac_sparse.next_element();
			}
			if( user_j == 0 )
				user_state = user_start;
			break;

			case UsrrpOp:
			// parameter result in an atomic operation sequence
			CPPAD_ASSERT_UNKNOWN( user_state == user_ret );
			CPPAD_ASSERT_UNKNOWN( 0 < user_i && user_i <= user_m );
			CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 );
			CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_par );
			--user_i;
			if( user_i == 0 )
				user_state = user_arg;
			break;

			case UsrrvOp:
			// variable result in an atomic operation sequence
			CPPAD_ASSERT_UNKNOWN( user_state == user_ret );
			CPPAD_ASSERT_UNKNOWN( 0 < user_i && user_i <= user_m );
			--user_i;
			if( RevJac[i_var] )
			{
				user_s[user_i] = true;
			}
			rev_hes_sparse.begin(i_var);
			j = rev_hes_sparse.next_element();
			while( j < user_q )
			{	if( user_pack )
					pack_u[user_i * user_q + j] = true;
				if( user_bool )
					bool_u[user_i * user_q + j] = true;
				if( user_set )
					set_u[user_i].insert(j);
				j = rev_hes_sparse.next_element();
			}
			if( user_i == 0 )
				user_state = user_arg;
			break;
			// -------------------------------------------------

			case ZmulpvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1)
			reverse_sparse_hessian_linear_unary_op(
			i_var, arg[1], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case ZmulvpOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1)
			reverse_sparse_hessian_linear_unary_op(
			i_var, arg[0], RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;
			// -------------------------------------------------

			case ZmulvvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1)
			reverse_sparse_hessian_mul_op(
			i_var, arg, RevJac, for_jac_sparse, rev_hes_sparse
			);
			break;

			// -------------------------------------------------

			default:
			CPPAD_ASSERT_UNKNOWN(0);
		}
# if CPPAD_REV_HES_SWEEP_TRACE
		for(j = 0; j < limit; j++)
		{	zf_value[j] = false;
			zh_value[j] = false;
		}
		for_jac_sparse.begin(i_var);;
		j = for_jac_sparse.next_element();;
		while( j < limit )
		{	zf_value[j] = true;
			j = for_jac_sparse.next_element();
		}
		rev_hes_sparse.begin(i_var);;
		j = rev_hes_sparse.next_element();;
		while( j < limit )
		{	zh_value[j] = true;
			j = rev_hes_sparse.next_element();
		}
		printOp(
			std::cout,
			play,
			i_op,
			i_var,
			op,
			arg
		);
		// should also print RevJac[i_var], but printOpResult does not
		// yet allow for this
		if( NumRes(op) > 0 && op != BeginOp ) printOpResult(
			std::cout,
			1,
			&zf_value,
			1,
			&zh_value
		);
		std::cout << std::endl;
	}
	std::cout << std::endl;
# else
	}
# endif
	// values corresponding to BeginOp
	CPPAD_ASSERT_UNKNOWN( i_op == 0 );
	CPPAD_ASSERT_UNKNOWN( i_var == 0 );

	return;
}
} // END_CPPAD_NAMESPACE

// preprocessor symbols that are local to this file
# undef CPPAD_REV_HES_SWEEP_TRACE

# endif
