// $Id$

# ifndef CPPAD_LOCAL_OPTIMIZE_OPTIMIZE_RUN_HPP
# define CPPAD_LOCAL_OPTIMIZE_OPTIMIZE_RUN_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    Eclipse Public License Version 1.0.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

# include <stack>
# include <iterator>
# include <cppad/local/optimize/usage.hpp>
# include <cppad/local/optimize/get_op_info.hpp>
# include <cppad/local/optimize/old2new.hpp>
# include <cppad/local/optimize/size_pair.hpp>
# include <cppad/local/optimize/csum_variable.hpp>
# include <cppad/local/optimize/csum_stacks.hpp>
# include <cppad/local/optimize/cskip_info.hpp>
# include <cppad/local/optimize/match_op.hpp>
# include <cppad/local/optimize/record_pv.hpp>
# include <cppad/local/optimize/record_vp.hpp>
# include <cppad/local/optimize/record_vv.hpp>
# include <cppad/local/optimize/record_csum.hpp>

/*!
\file optimize_run.hpp
Convert a player object to an optimized recorder object
*/
// BEGIN_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
namespace CppAD { namespace local { namespace optimize  {
/*!
Convert a player object to an optimized recorder object

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< \a Base > and computations by this routine are done using type
\a Base.


\param options
\li
If the sub-string "no_conditional_skip" appears,
conditional skip operations will not be generated.
This may make the optimize routine use significantly less memory
and take significantly less time.
\li
If the sub-string "no_compare_op" appears,
then comparison operators will be removed from the optimized tape.
These operators are necessary for the compare_change function to be
be meaningful in the resulting recording.
On the other hand, they are not necessary and take extra time
when compare_change is not used.
\li
If the sub-string "no_print_for" appears,
then print forward (PriOp) operators will be removed from the optimized tape.
These operators are useful for reporting problems evaluating derivatives
at independent variable values different from those used to record a function.

\param n
is the number of independent variables on the tape.

\param dep_taddr
On input this vector contains the indices for each of the dependent
variable values in the operation sequence corresponding to \a play.
Upon return it contains the indices for the same variables but in
the operation sequence corresponding to \a rec.

\param play
This is the operation sequence that we are optimizing.
It is essentially const, except for play back state which
changes while it plays back the operation seqeunce.

\param rec
The input contents of this recording does not matter.
Upon return, it contains an optimized verison of the
operation sequence corresponding to \a play.
*/

template <class Base>
void optimize_run(
	const std::string&           options   ,
	size_t                       n         ,
	CppAD::vector<size_t>&       dep_taddr ,
	player<Base>*                play      ,
	recorder<Base>*              rec       )
{
	bool conditional_skip = true;
	bool compare_op       = true;
	bool print_for_op     = true;
	size_t index = 0;
	while( index < options.size() )
	{	while( index < options.size() && options[index] == ' ' )
			++index;
		std::string option;
		while( index < options.size() && options[index] != ' ' )
			option += options[index++];
		if( option != "" )
		{	if( option == "no_conditional_skip" )
				conditional_skip = false;
			else if( option == "no_compare_op" )
				compare_op = false;
			else if( option == "no_print_for_op" )
				print_for_op = false;
			else
			{	option += " is not a valid optimize option";
				CPPAD_ASSERT_KNOWN( false , option.c_str() );
			}
		}
	}
	// number of operators in the player
	const size_t num_op = play->num_op_rec();
	CPPAD_ASSERT_UNKNOWN(
		num_op < size_t( std::numeric_limits<addr_t>::max() )
	);

	// number of variables in the player
	const size_t num_var = play->num_var_rec();

	// number of  VecAD indices
	size_t num_vecad_ind   = play->num_vec_ind_rec();

	// number of VecAD vectors
	size_t num_vecad_vec   = play->num_vecad_vec_rec();

	// operator information
	vector<addr_t>            var2op;
	vector<struct_cskip_info> cskip_info;
	vector<bool>              vecad_used;
	vector<struct_op_info>    op_info;
	get_op_info(
		conditional_skip,
		compare_op,
		print_for_op,
		play,
		dep_taddr,
		var2op,
		cskip_info,
		vecad_used,
		op_info
	);

	// nan with type Base
	Base base_nan = Base( std::numeric_limits<double>::quiet_NaN() );

	// -------------------------------------------------------------
	// information for current operator
	size_t        i_op;   // index
	OpCode        op;     // operator
	const addr_t* arg;    // arguments
	size_t        i_var;  // variable index of primary (last) result

	enum_user_state user_state;
	// -------------------------------------------------------------
	// conditional skip information
	//
	// size of the conditional cskip information structure
	// (This is equal to the number of conditional expressions when
	// conditional_skip is true.)
	size_t num_cskip = cskip_info.size();
	CPPAD_ASSERT_UNKNOWN( conditional_skip || num_cskip == 0 );

	// sort the conditional skip information by max_left_right
	vector<size_t> cskip_info_order(num_cskip);
	if( num_cskip > 0 )
	{	CppAD::vector<size_t> keys(num_cskip);
		for(size_t i = 0; i < num_cskip; i++)
			keys[i] = cskip_info[i].max_left_right;
		CppAD::index_sort(keys, cskip_info_order);
	}

	// index in sorted order
	size_t cskip_order_next = 0;

	// index in order during reverse sweep
	size_t cskip_info_index = num_cskip;
	vector<struct_cskip_new> cskip_new(num_cskip);
	// flag used to indicate that this conditional expression is skipped
	for(size_t i = 0; i < num_cskip; i++)
		cskip_new[i].i_arg = 0;
	// -------------------------------------------------------------

	// Erase all information in the old recording
	rec->free();

	// initialize mapping from old VecAD index to new VecAD index
	CppAD::vector<size_t> new_vecad_ind(num_vecad_ind);
	for(size_t i = 0; i < num_vecad_ind; i++)
		new_vecad_ind[i] = num_vecad_ind; // invalid index
	{
		size_t j = 0;     // index into the old set of indices
		for(size_t i = 0; i < num_vecad_vec; i++)
		{	// length of this VecAD
			size_t length = play->GetVecInd(j);
			if( vecad_used[i] )
			{	// Put this VecAD vector in new recording
				CPPAD_ASSERT_UNKNOWN(length < num_vecad_ind);
				new_vecad_ind[j] = rec->PutVecInd(length);
				for(size_t k = 1; k <= length; k++) new_vecad_ind[j+k] =
					rec->PutVecInd(
						rec->PutPar(
							play->GetPar(
								play->GetVecInd(j+k)
				) ) );
			}
			// start of next VecAD
			j       += length + 1;
		}
		CPPAD_ASSERT_UNKNOWN( j == num_vecad_ind );
	}
	//
	// Mapping from old operator index to new operator information
	// (zero is invalid except for old2new[0].new_op and old2new[0].i_var)
	vector<struct_old2new> old2new(num_op);
	for(size_t i = 0; i < num_op; i++)
	{	old2new[i].new_op  = 0;
		old2new[i].new_var = 0;
	}


	// temporary buffer for new argument values
	addr_t new_arg[6];

	// temporary work space used by record_csum
	// (decalared here to avoid realloaction of memory)
	struct_csum_stacks csum_work;

	// tempory used to hold a size_pair
	struct_size_pair size_pair;

	user_state = start_user;
	for(i_op = 0; i_op < num_op; ++i_op)
	{	addr_t mask; // temporary used in some switch cases
		//
		// this operator information
		op    =  op_info[i_op].op;
		arg   =  op_info[i_op].arg;
		i_var =  op_info[i_op].i_var;
		//
		// determine if we should insert a conditional skip here
		bool skip  = conditional_skip;
		skip      &= cskip_order_next < num_cskip;
		skip      &= op != BeginOp;
		skip      &= op != InvOp;
		skip      &= user_state == start_user;
		if( skip )
		{	size_t j = cskip_info_order[cskip_order_next];
			if( NumRes(op) > 0 )
				skip &= cskip_info[j].max_left_right < i_var;
			else
				skip &= cskip_info[j].max_left_right <= i_var;
		}
		if( skip )
		{	size_t j = cskip_info_order[cskip_order_next];
			cskip_order_next++;
			struct_cskip_info info = cskip_info[j];
			size_t n_true          = info.skip_op_true.size();
			size_t n_false         = info.skip_op_false.size();
			skip &= n_true > 0 || n_false > 0;
			if( skip )
			{	CPPAD_ASSERT_UNKNOWN( NumRes(CSkipOp) == 0 );
				size_t n_arg   = 7 + n_true + n_false;
				// reserve space for the arguments to this operator but
				// delay setting them until we have all the new addresses
				cskip_new[j].i_arg = rec->ReserveArg(n_arg);
				// i_arg == 0 is used to check if conditional expression
				// has been skipped.
				CPPAD_ASSERT_UNKNOWN( cskip_new[j].i_arg > 0 );
				// There is no corresponding old operator in this case
				rec->PutOp(CSkipOp);
			}
		}
		if( op == UserOp )
		{	if( user_state == start_user )
				user_state = end_user;
			else
			{	CPPAD_ASSERT_UNKNOWN( user_state == end_user );
				user_state = start_user;
			}
		}
		size_t         previous;
		//
		if( op_info[i_op].usage == yes_usage ) switch( op )
		{
			case BeginOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1);
			// Put BeginOp at beginning of recording
			old2new[i_op].new_op  = rec->num_op_rec();
			old2new[i_op].new_var = rec->PutOp(BeginOp);
			rec->PutArg(arg[0]);
			break;

			// --------------------------------------------------------------
			// Unary operator where operand is arg[0]
			case AbsOp:
			case AcosOp:
			case AcoshOp:
			case AsinOp:
			case AsinhOp:
			case AtanOp:
			case AtanhOp:
			case CosOp:
			case CoshOp:
			case ErfOp:
			case ExpOp:
			case Expm1Op:
			case LogOp:
			case Log1pOp:
			case SignOp:
			case SinOp:
			case SinhOp:
			case SqrtOp:
			case TanOp:
			case TanhOp:
			previous = op_info[i_op].previous;
			if( previous > 0 )
			{	size_t j_op = previous;
				old2new[i_op].new_var = old2new[j_op].new_var;
			}
			else
			{	//
				new_arg[0]   = old2new[ var2op[arg[0]] ].new_var;
				rec->PutArg( new_arg[0] );
				//
				old2new[i_op].new_op  = rec->num_op_rec();
				old2new[i_op].new_var = rec->PutOp(op);
				CPPAD_ASSERT_UNKNOWN(
					new_arg[0] < old2new[var2op[i_var]].new_var
				);
				if( op == ErfOp )
				{	// Error function is a special case
					// second argument is always the parameter 0
					// third argument is always the parameter 2 / sqrt(pi)
					CPPAD_ASSERT_UNKNOWN( NumArg(ErfOp) == 3 );
					rec->PutArg( rec->PutPar( Base(0) ) );
					rec->PutArg( rec->PutPar(
						Base( 1.0 / std::sqrt( std::atan(1.0) ) )
					) );
				}
			}
			break;
			// ---------------------------------------------------
			// Binary operators where
			// left is a variable and right is a parameter
			case SubvpOp:
			// check if this is the top of a csum connection
			if( op_info[i_op].usage == csum_usage )
				break;
			if( op_info[ var2op[arg[0]] ].usage == csum_usage )
			{
				// convert to a sequence of summation operators
				size_pair = record_csum(
					var2op              ,
					op_info             ,
					old2new             ,
					i_var               ,
					play->num_par_rec() ,
					play->GetPar()      ,
					rec                 ,
					csum_work
				);
				old2new[i_op].new_op  = size_pair.i_op;
				old2new[i_op].new_var = size_pair.i_var;
				// abort rest of this case
				break;
			}
			case DivvpOp:
			case PowvpOp:
			case ZmulvpOp:
			previous = op_info[i_op].previous;
			if( previous > 0 )
			{	size_t j_op = previous;
				old2new[i_op].new_var = old2new[j_op].new_var;
			}
			else
			{	//
				size_pair = record_vp(
					var2op              ,
					op_info             ,
					old2new             ,
					i_var               ,
					play->num_par_rec() ,
					play->GetPar()      ,
					rec                 ,
					op                  ,
					arg
				);
				old2new[i_op].new_op  = size_pair.i_op;
				old2new[i_op].new_var = size_pair.i_var;
			}
			break;
			// ---------------------------------------------------
			// Binary operators where
			// left is an index and right is a variable
			case DisOp:
			previous = op_info[i_op].previous;
			if( previous > 0 )
			{	size_t j_op = previous;
				old2new[i_op].new_var = old2new[j_op].new_var;
			}
			else
			{	//
				new_arg[0] = arg[0];
				new_arg[1] = old2new[ var2op[arg[1]] ].new_var;
				rec->PutArg( new_arg[0], new_arg[1] );
				//
				old2new[i_op].new_op  = rec->num_op_rec();
				old2new[i_op].new_var = rec->PutOp(op);
				CPPAD_ASSERT_UNKNOWN(
					new_arg[1] < old2new[var2op[i_var]].new_var
				);
			}
			break;

			// ---------------------------------------------------
			// Binary operators where
			// left is a parameter and right is a variable
			case SubpvOp:
			case AddpvOp:
			// check if this is the top of a csum connection
			if( op_info[i_op].usage == csum_usage )
				break;
			if( op_info[ var2op[arg[1]] ].usage == csum_usage )
			{
				// convert to a sequence of summation operators
				size_pair = record_csum(
					var2op              ,
					op_info             ,
					old2new             ,
					i_var               ,
					play->num_par_rec() ,
					play->GetPar()      ,
					rec                 ,
					csum_work
				);
				old2new[i_op].new_op  = size_pair.i_op;
				old2new[i_op].new_var = size_pair.i_var;
				// abort rest of this case
				break;
			}
			case DivpvOp:
			case MulpvOp:
			case PowpvOp:
			case ZmulpvOp:
			previous = op_info[i_op].previous;
			if( previous > 0 )
			{	size_t j_op = previous;
				old2new[i_op].new_var = old2new[j_op].new_var;
			}
			else
			{	//
				size_pair = record_pv(
					var2op              ,
					op_info             ,
					old2new             ,
					i_var               ,
					play->num_par_rec() ,
					play->GetPar()      ,
					rec                 ,
					op                  ,
					arg
				);
				old2new[i_op].new_op  = size_pair.i_op;
				old2new[i_op].new_var = size_pair.i_var;
			}
			break;
			// ---------------------------------------------------
			// Binary operator where both operands are variables
			case AddvvOp:
			case SubvvOp:
			// check if this is the top of a csum connection
			if( op_info[i_op].usage == csum_usage )
				break;
			if(
				op_info[ var2op[arg[0]] ].usage == csum_usage ||
				op_info[ var2op[arg[1]] ].usage == csum_usage
			)
			{
				// convert to a sequence of summation operators
				size_pair = record_csum(
					var2op              ,
					op_info             ,
					old2new             ,
					i_var               ,
					play->num_par_rec() ,
					play->GetPar()      ,
					rec                 ,
					csum_work
				);
				old2new[i_op].new_op  = size_pair.i_op;
				old2new[i_op].new_var = size_pair.i_var;
				// abort rest of this case
				break;
			}
			case DivvvOp:
			case MulvvOp:
			case PowvvOp:
			case ZmulvvOp:
			previous = op_info[i_op].previous;
			if( previous > 0 )
			{	size_t j_op = previous;
				old2new[i_op].new_var = old2new[j_op].new_var;
			}
			else
			{	//
				size_pair = record_vv(
					var2op              ,
					op_info             ,
					old2new             ,
					i_var               ,
					play->num_par_rec() ,
					play->GetPar()      ,
					rec                 ,
					op                  ,
					arg
				);
				old2new[i_op].new_op  = size_pair.i_op;
				old2new[i_op].new_var = size_pair.i_var;
			}
			break;
			// ---------------------------------------------------
			// Conditional expression operators
			case CExpOp:
			CPPAD_ASSERT_NARG_NRES(op, 6, 1);
			new_arg[0] = arg[0];
			new_arg[1] = arg[1];
			mask = 1;
			for(size_t i = 2; i < 6; i++)
			{	if( arg[1] & mask )
				{	new_arg[i] = old2new[ var2op[arg[i]] ].new_var;
					CPPAD_ASSERT_UNKNOWN(
						size_t(new_arg[i]) < num_var
					);
				}
				else	new_arg[i] = rec->PutPar(
						play->GetPar( arg[i] )
				);
				mask = mask << 1;
			}
			rec->PutArg(
				new_arg[0] ,
				new_arg[1] ,
				new_arg[2] ,
				new_arg[3] ,
				new_arg[4] ,
				new_arg[5]
			);
			old2new[i_op].new_op  = rec->num_op_rec();
			old2new[i_op].new_var = rec->PutOp(op);
			//
			// The new addresses for left and right are used during
			// fill in the arguments for the CSkip operations. This does not
			// affect max_left_right which is used during this sweep.
			if( conditional_skip )
			{	CPPAD_ASSERT_UNKNOWN( cskip_info_index > 0 );
				cskip_info_index--;
				cskip_new[ cskip_info_index ].left  = new_arg[2];
				cskip_new[ cskip_info_index ].right = new_arg[3];
			}
			break;
			// ---------------------------------------------------
			// Operations with no arguments and no results
			case EndOp:
			CPPAD_ASSERT_NARG_NRES(op, 0, 0);
			old2new[i_op].new_op = rec->num_op_rec();
			rec->PutOp(op);
			break;
			// ---------------------------------------------------
			// Operations with two arguments and no results
			case LepvOp:
			case LtpvOp:
			case EqpvOp:
			case NepvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 0);
			new_arg[0] = rec->PutPar( play->GetPar(arg[0]) );
			new_arg[1] = old2new[ var2op[arg[1]] ].new_var;
			rec->PutArg(new_arg[0], new_arg[1]);
			old2new[i_op].new_op = rec->num_op_rec();
			rec->PutOp(op);
			break;
			//
			case LevpOp:
			case LtvpOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 0);
			new_arg[0] = old2new[ var2op[arg[0]] ].new_var;
			new_arg[1] = rec->PutPar( play->GetPar(arg[1]) );
			rec->PutArg(new_arg[0], new_arg[1]);
			old2new[i_op].new_op = rec->num_op_rec();
			rec->PutOp(op);
			break;
			//
			case LevvOp:
			case LtvvOp:
			case EqvvOp:
			case NevvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 0);
			new_arg[0] = old2new[ var2op[arg[0]] ].new_var;
			new_arg[1] = old2new[ var2op[arg[1]] ].new_var;
			rec->PutArg(new_arg[0], new_arg[1]);
			old2new[i_op].new_op = rec->num_op_rec();
			rec->PutOp(op);
			break;

			// ---------------------------------------------------
			// Operations with no arguments and one result
			case InvOp:
			CPPAD_ASSERT_NARG_NRES(op, 0, 1);
			old2new[i_op].new_op  = rec->num_op_rec();
			old2new[i_op].new_var = rec->PutOp(op);
			break;
			// ---------------------------------------------------
			// Operations with one argument that is a parameter
			case ParOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1);
			new_arg[0] = rec->PutPar( play->GetPar(arg[0] ) );
			rec->PutArg( new_arg[0] );
			//
			old2new[i_op].new_op  = rec->num_op_rec();
			old2new[i_op].new_var = rec->PutOp(op);
			break;

			// ---------------------------------------------------
			// print forward operator
			case PriOp:
			CPPAD_ASSERT_NARG_NRES(op, 5, 0);
			// arg[0]
			new_arg[0] = arg[0];
			//
			// arg[1]
			if( arg[0] & 1 )
			{	new_arg[1] = old2new[ var2op[arg[1]] ].new_var;
				CPPAD_ASSERT_UNKNOWN( size_t(new_arg[1]) < num_var );
			}
			else
			{	new_arg[1] = rec->PutPar( play->GetPar( arg[1] ) );
			}
			//
			// arg[3]
			if( arg[0] & 2 )
			{	new_arg[3] = old2new[ var2op[arg[3]] ].new_var;
				CPPAD_ASSERT_UNKNOWN( size_t(new_arg[3]) < num_var );
			}
			else
			{	new_arg[3] = rec->PutPar( play->GetPar( arg[3] ) );
			}
			new_arg[2] = rec->PutTxt( play->GetTxt(arg[2]) );
			new_arg[4] = rec->PutTxt( play->GetTxt(arg[4]) );
			//
			rec->PutArg(
				new_arg[0] ,
				new_arg[1] ,
				new_arg[2] ,
				new_arg[3] ,
				new_arg[4]
			);
			// new operator
			old2new[i_op].new_op  = rec->num_op_rec();
			// no new variable
			rec->PutOp(op);
			break;

			// ---------------------------------------------------
			// VecAD operators

			// Load using a parameter index
			case LdpOp:
			CPPAD_ASSERT_NARG_NRES(op, 3, 1);
			new_arg[0] = new_vecad_ind[ arg[0] ];
			new_arg[1] = arg[1];
			new_arg[2] = rec->num_load_op_rec();
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < num_vecad_ind );
			rec->PutArg(
				new_arg[0],
				new_arg[1],
				new_arg[2]
			);
			old2new[i_op].new_op  = rec->num_op_rec();
			old2new[i_op].new_var = rec->PutLoadOp(op);
			break;

			// Load using a variable index
			case LdvOp:
			CPPAD_ASSERT_NARG_NRES(op, 3, 1);
			new_arg[0] = new_vecad_ind[ arg[0] ];
			new_arg[1] = old2new[ var2op[arg[1]] ].new_var;
			new_arg[2] = rec->num_load_op_rec();
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < num_vecad_ind );
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[1]) < num_var );
			rec->PutArg(
				new_arg[0],
				new_arg[1],
				new_arg[2]
			);
			old2new[i_op].new_op  = rec->num_op_rec();
			old2new[i_op].new_var = rec->PutLoadOp(op);
			break;

			// Store a parameter using a parameter index
			case StppOp:
			CPPAD_ASSERT_NARG_NRES(op, 3, 0);
			new_arg[0] = new_vecad_ind[ arg[0] ];
			new_arg[1] = rec->PutPar( play->GetPar(arg[1]) );
			new_arg[2] = rec->PutPar( play->GetPar(arg[2]) );
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < num_vecad_ind );
			rec->PutArg(
				new_arg[0],
				new_arg[1],
				new_arg[2]
			);
			old2new[i_op].new_op = rec->num_op_rec();
			rec->PutOp(op);
			break;

			// Store a parameter using a variable index
			case StvpOp:
			CPPAD_ASSERT_NARG_NRES(op, 3, 0);
			new_arg[0] = new_vecad_ind[ arg[0] ];
			new_arg[1] = old2new[ var2op[arg[1]] ].new_var;
			new_arg[2] = rec->PutPar( play->GetPar(arg[2]) );
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < num_vecad_ind );
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[1]) < num_var );
			rec->PutArg(
				new_arg[0],
				new_arg[1],
				new_arg[2]
			);
			old2new[i_op].new_op = rec->num_op_rec();
			rec->PutOp(op);
			break;

			// Store a variable using a parameter index
			case StpvOp:
			CPPAD_ASSERT_NARG_NRES(op, 3, 0);
			new_arg[0] = new_vecad_ind[ arg[0] ];
			new_arg[1] = rec->PutPar( play->GetPar(arg[1]) );
			new_arg[2] = old2new[ var2op[arg[2]] ].new_var;
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < num_vecad_ind );
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[1]) < num_var );
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[2]) < num_var );
			rec->PutArg(
				new_arg[0],
				new_arg[1],
				new_arg[2]
			);
			old2new[i_op].new_op = rec->num_op_rec();
			rec->PutOp(op);
			break;

			// Store a variable using a variable index
			case StvvOp:
			CPPAD_ASSERT_NARG_NRES(op, 3, 0);
			new_arg[0] = new_vecad_ind[ arg[0] ];
			new_arg[1] = old2new[ var2op[arg[1]] ].new_var;
			new_arg[2] = old2new[ var2op[arg[2]] ].new_var;
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < num_vecad_ind );
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[1]) < num_var );
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[2]) < num_var );
			rec->PutArg(
				new_arg[0],
				new_arg[1],
				new_arg[2]
			);
			old2new[i_op].new_op = rec->num_op_rec();
			rec->PutOp(op);
			break;

			// -----------------------------------------------------------
			// user atomic function call operators

			case UserOp:
			CPPAD_ASSERT_NARG_NRES(op, 4, 0);
			// user_old, user_n, user_m
			rec->PutArg(arg[0], arg[1], arg[2], arg[3]);
			old2new[i_op].new_op = rec->num_op_rec();
			rec->PutOp(UserOp);
			break;

			case UsrapOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 0);
			new_arg[0] = rec->PutPar( play->GetPar(arg[0]) );
			rec->PutArg(new_arg[0]);
			old2new[i_op].new_op = rec->num_op_rec();
			rec->PutOp(UsrapOp);
			break;

			case UsravOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 0);
			new_arg[0] = old2new[ var2op[arg[0]] ].new_var;
			if( size_t(new_arg[0]) < num_var )
			{	rec->PutArg(new_arg[0]);
				old2new[i_op].new_op = rec->num_op_rec();
				rec->PutOp(UsravOp);
			}
			else
			{	// This argument does not affect the result and
				// has been optimized out so use nan in its place.
				new_arg[0] = rec->PutPar( base_nan );
				rec->PutArg(new_arg[0]);
				old2new[i_op].new_op = rec->num_op_rec();
				rec->PutOp(UsrapOp);
			}
			break;

			case UsrrpOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 0);
			new_arg[0] = rec->PutPar( play->GetPar(arg[0]) );
			rec->PutArg(new_arg[0]);
			old2new[i_op].new_op = rec->num_op_rec();
			rec->PutOp(UsrrpOp);
			break;

			case UsrrvOp:
			CPPAD_ASSERT_NARG_NRES(op, 0, 1);
			old2new[i_op].new_op  = rec->num_op_rec();
			old2new[i_op].new_var = rec->PutOp(UsrrvOp);
			break;
			// ---------------------------------------------------

			// all cases should be handled above
			default:
			CPPAD_ASSERT_UNKNOWN(false);

		}
	}
	// modify the dependent variable vector to new indices
	for(size_t i = 0; i < dep_taddr.size(); i++ )
	{	dep_taddr[i] = old2new[ var2op[dep_taddr[i]] ].new_var;
		CPPAD_ASSERT_UNKNOWN( size_t(dep_taddr[i]) < num_var );
	}

# ifndef NDEBUG
	for(i_op = 0; i_op < num_op; i_op++)
		if( NumRes( op_info[i_op].op ) > 0 )
			CPPAD_ASSERT_UNKNOWN(
				size_t(old2new[i_op].new_op) < rec->num_op_rec()
			);
# endif
	// make sure that all the conditional expressions have been
	// checked to see if they are still present
	CPPAD_ASSERT_UNKNOWN( cskip_order_next == num_cskip );
	// fill in the arguments for the CSkip operations
	for(size_t i = 0; i < num_cskip; i++)
	{	// if cskip_new[i].i_arg == 0, this conditional expression was skipped
		if( cskip_new[i].i_arg > 0 )
		{	struct_cskip_info info = cskip_info[i];
			size_t n_true  = info.skip_op_true.size();
			size_t n_false = info.skip_op_false.size();
			size_t i_arg   = cskip_new[i].i_arg;
			rec->ReplaceArg(i_arg++, info.cop   );
			rec->ReplaceArg(i_arg++, info.flag  );
			rec->ReplaceArg(i_arg++, info.left  );
			rec->ReplaceArg(i_arg++, info.right );
			rec->ReplaceArg(i_arg++, n_true     );
			rec->ReplaceArg(i_arg++, n_false    );
			for(size_t j = 0; j < info.skip_op_true.size(); j++)
			{	i_op = cskip_info[i].skip_op_true[j];
				// op_info[i_op].usage == yes_usage
				CPPAD_ASSERT_UNKNOWN( old2new[i_op].new_op != 0 );
				rec->ReplaceArg(i_arg++, old2new[i_op].new_op );
			}
			for(size_t j = 0; j < info.skip_op_false.size(); j++)
			{	i_op   = cskip_info[i].skip_op_false[j];
				// op_info[i_op].usage == yes_usage
				CPPAD_ASSERT_UNKNOWN( old2new[i_op].new_op != 0 );
				rec->ReplaceArg(i_arg++, old2new[i_op].new_op );
			}
			rec->ReplaceArg(i_arg++, n_true + n_false);
# ifndef NDEBUG
			size_t n_arg   = 7 + n_true + n_false;
			CPPAD_ASSERT_UNKNOWN( cskip_new[i].i_arg + n_arg == i_arg );
# endif
		}
	}
}

} } } // END_CPPAD_LOCAL_OPTIMIZE_NAMESPACE

# endif
