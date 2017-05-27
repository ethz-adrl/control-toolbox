// $Id: optimize.hpp 3757 2015-11-30 12:03:07Z bradbell $
# ifndef CPPAD_OPTIMIZE_HPP
# define CPPAD_OPTIMIZE_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    Eclipse Public License Version 1.0.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

/*
$begin optimize$$
$spell
	enum
	jac
	bool
	Taylor
	var
	CppAD
	cppad
	std
	CondExpEq
$$

$section Optimize an ADFun Object Tape$$
$mindex sequence operations speed memory NDEBUG$$


$head Syntax$$
$icode%f%.optimize()%$$


$head Purpose$$
The operation sequence corresponding to an $cref ADFun$$ object can
be very large and involve many operations; see the
size functions in $cref seq_property$$.
The $icode%f%.optimize%$$ procedure reduces the number of operations,
and thereby the time and the memory, required to
compute function and derivative values.

$head f$$
The object $icode f$$ has prototype
$codei%
	ADFun<%Base%> %f%
%$$

$head Improvements$$
You can see the reduction in number of variables in the operation sequence
by calling the function $cref/f.size_var()/seq_property/size_var/$$
before and after the optimization procedure.
Given that the optimization procedure takes time,
it may be faster to skip this optimize procedure and just compute
derivatives using the original operation sequence.

$subhead Testing$$
You can run the CppAD $cref/speed/speed_main/$$ tests and see
the corresponding changes in number of variables and execution time;
see $cref cmake_check$$.

$head Efficiency$$
The $code optimize$$ member function
may greatly reduce the number of variables
in the operation sequence; see $cref/size_var/seq_property/size_var/$$.
If a $cref/zero order forward/forward_zero/$$ calculation is done during
the construction of $icode f$$, it will require more memory
and time than required after the optimization procedure.
In addition, it will need to be redone.
For this reason, it is more efficient to use
$codei%
	ADFun<%Base%> %f%;
	%f%.Dependent(%x%, %y%);
	%f%.optimize();
%$$
instead of
$codei%
	ADFun<%Base%> %f%(%x%, %y%)
	%f%.optimize();
%$$
See the discussion about
$cref/sequence constructors/FunConstruct/Sequence Constructor/$$.

$head Atomic Functions$$
There are some subtitle issue with optimized $cref atomic$$ functions
$latex v = g(u)$$:

$subhead rev_sparse_jac$$
The $cref atomic_rev_sparse_jac$$ function is be used to determine
which components of $icode u$$ affect the dependent variables of $icode f$$.
For each atomic operation, the current
$cref/atomic_sparsity/atomic_option/atomic_sparsity/$$ setting is used
to determine if $code pack_sparsity_enum$$, $code bool_sparsity_enum$$,
or $code set_sparsity_enum$$ is used to determine dependency relations
between argument and result variables.

$subhead nan$$
If $icode%u%[%i%]%$$ does not affect the value of
the dependent variables for $icode f$$,
the value of $icode%u%[%i%]%$$ is set to $cref nan$$.


$head Checking Optimization$$
If $cref/NDEBUG/Faq/Speed/NDEBUG/$$ is not defined,
and $cref/f.size_order()/size_order/$$ is greater than zero,
a $cref forward_zero$$ calculation is done using the optimized version
of $icode f$$ and the results are checked to see that they are
the same as before.
If they are not the same, the
$cref ErrorHandler$$ is called with a known error message
related to $icode%f%.optimize()%$$.

$head Example$$
$children%
	example/optimize.cpp
%$$
The file
$cref optimize.cpp$$
contains an example and test of this operation.
It returns true if it succeeds and false otherwise.

$end
-----------------------------------------------------------------------------
*/
# include <stack>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
namespace optimize { // BEGIN_CPPAD_OPTIMIZE_NAMESPACE
/*!
\file optimize.hpp
Routines for optimizing a tape
*/


/*!
State for this variable set during reverse sweep.
*/
enum enum_connect_type {
	/// There is no operation that connects this variable to the
	/// independent variables.
	not_connected        ,

	/// There is one or more operations that connects this variable to the
	/// independent variables.
	yes_connected        ,

	/// There is only one parrent that connects this variable to the
	/// independent variables and the parent is a summation operation; i.e.,
	/// AddvvOp, AddpvOp, SubpvOp, SubvpOp, or SubvvOp.
	sum_connected        ,

	/// Satisfies the sum_connected assumptions above and in addition
	/// this variable is the result of summation operator.
	csum_connected       ,

	/// This node is only connected in the case where the comparision is
	/// true for the conditional expression with index \c connect_index.
	cexp_connected

};

/*!
Class used to hold information about one conditional expression.
*/
class class_cexp_pair {
public:
	/// packs both the compare and index information
	/// compare = pack_ % 2
	/// index   = pack_ / 2
	size_t pack_;

	/// If this is true (false) this connection is only for the case where
	/// the comparision in the conditional expression is true (false)
	bool compare(void) const
	{	return pack_ % 2 != 0; }

	/// This is the index of the conditional expression (in cksip_info)
	/// for this connection
	size_t index(void) const
	{	return pack_ / 2; }

	/// constructor
	class_cexp_pair(const bool& compare_arg, const size_t& index_arg)
	: pack_(size_t(compare_arg) + 2 * index_arg )
	{	CPPAD_ASSERT_UNKNOWN( compare_arg == compare() );
		CPPAD_ASSERT_UNKNOWN( index_arg == index() );
	}

	/// assignment operator
	void operator=(const class_cexp_pair& right)
	{	pack_ = right.pack_; }

	/// not equal operator
	bool operator!=(const class_cexp_pair& right)
	{	return pack_ != right.pack_; }

	/// Less than operator
	/// (required for intersection of two sets of class_cexp_pair elements).
	bool operator<(const class_cexp_pair& right) const
	{	return pack_ < right.pack_; }
};

/*!
A container that is like std::set<class_cexp_pair> except that it does
not allocate empty sets and only has a few operations.
*/
class class_set_cexp_pair {
private:
	// This set is empty if and only if ptr_ == CPPAD_NULL;
	std::set<class_cexp_pair>* ptr_;

	void new_ptr(void)
	{	CPPAD_ASSERT_UNKNOWN( ptr_ == CPPAD_NULL );
		ptr_ = new std::set<class_cexp_pair>;
		CPPAD_ASSERT_UNKNOWN( ptr_ != CPPAD_NULL );
		// std::cout << "new ptr_ = " << ptr_ << std::endl;
	}

	void delete_ptr(void)
	{	if( ptr_ != CPPAD_NULL )
		{	// std::cout << "delete ptr_ = " << ptr_ << std::endl;
			delete ptr_;
		}
		ptr_ = CPPAD_NULL;
	}

public:
	/// constructor
	class_set_cexp_pair(void)
	{	ptr_ = CPPAD_NULL; }

	/// destructor
	~class_set_cexp_pair(void)
	{	delete_ptr(); }

	void print(void)
	{	if( ptr_ == CPPAD_NULL )
		{	std::cout << "{ }";
			return;
		}
		CPPAD_ASSERT_UNKNOWN( ! empty() );
		const char* sep = "{ ";
		std::set<class_cexp_pair>::const_iterator itr;
		for(itr = ptr_->begin(); itr != ptr_->end(); itr++)
		{	std::cout << sep;
			std::cout << "(" << itr->compare() << "," << itr->index() << ")";
			sep = ", ";
		}
		std::cout << "}";
	}

	/// assignment operator
	void operator=(const class_set_cexp_pair& other)
	{	// make this a copy of the other set
		if( other.ptr_ == CPPAD_NULL )
		{	if( ptr_ == CPPAD_NULL )
				return;
			delete_ptr();
			return;
		}
		CPPAD_ASSERT_UNKNOWN( ! other.empty() );
		if( ptr_ == CPPAD_NULL )
			new_ptr();
		*ptr_ = *other.ptr_;
	}

	/// insert an element in this set
	void insert(const class_cexp_pair& element)
	{	if( ptr_ == CPPAD_NULL )
			new_ptr();
		ptr_->insert(element);
		CPPAD_ASSERT_UNKNOWN( ! empty() );
	}

	/// is this set empty
	bool empty(void) const
	{	if( ptr_ == CPPAD_NULL )
			return true;
		CPPAD_ASSERT_UNKNOWN( ! ptr_->empty() );
		return false;
	}

	/// remove the elements in this set
	void clear(void)
	{	if( ptr_ == CPPAD_NULL )
			return;
		CPPAD_ASSERT_UNKNOWN( ! empty() );
		delete_ptr();
	}

	// returns begin pointer for the set
	std::set<class_cexp_pair>::const_iterator begin(void)
	{	CPPAD_ASSERT_UNKNOWN( ! empty() );
		return ptr_->begin();
	}

	// returns end pointer for the set
	std::set<class_cexp_pair>::const_iterator end(void)
	{	CPPAD_ASSERT_UNKNOWN( ! empty() );
		return ptr_->end();
	}

	/*!
	Make this set the intersection of itself with another set.

	\param other
	the other set

	*/
	void intersection(const class_set_cexp_pair& other )
	{	// empty result case
		if( ptr_ == CPPAD_NULL )
			return;

		// empty result case
		if( other.ptr_ == CPPAD_NULL )
		{	delete_ptr();
			return;
		}

		// put result here
		class_set_cexp_pair result;
		CPPAD_ASSERT_UNKNOWN( result.ptr_ == CPPAD_NULL );
		result.new_ptr();
		CPPAD_ASSERT_UNKNOWN( result.ptr_ != CPPAD_NULL );

		// do the intersection
		std::set_intersection(
			ptr_->begin()   ,
			ptr_->end()     ,
			other.ptr_->begin()  ,
			other.ptr_->end()    ,
			std::inserter(*result.ptr_, result.ptr_->begin())
		);
		if( result.ptr_->empty() )
			result.delete_ptr();

		// swap this and the result
		std::swap(ptr_, result.ptr_);

		return;
	}

};
/*!
Structure used by \c optimize to hold information about one variable.
in the old operation seqeunce.
*/
struct struct_old_variable {
	/// Operator for which this variable is the result, \c NumRes(op) > 0.
	/// Set by the reverse sweep at beginning of optimization.
	OpCode              op;

	/// Pointer to first argument (child) for this operator.
	/// Set by the reverse sweep at beginning of optimization.
	const addr_t*       arg;

	/// How is this variable connected to the independent variables
	enum_connect_type connect_type;

	/// New operation sequence corresponding to this old varable.
	/// Set during forward sweep to the index in the new tape
	addr_t new_var;

	/// New operator index for this varable.
	/// Set during forward sweep to the index in the new tape
	size_t new_op;

	/// Did this variable match another variable in the operation sequence
	bool match;
};

struct struct_size_pair {
	size_t i_op;  // an operator index
	size_t i_var; // a variable index
};

/*!
Structures used by \c record_csum
to hold information about one variable.
*/
struct struct_csum_variable {
	/// Operator for which this variable is the result, \c NumRes(op) > 0.
	OpCode              op;

	/// Pointer to first argument (child) for this operator.
	/// Set by the reverse sweep at beginning of optimization.
	const addr_t*       arg;

	/// Is this variable added to the summation
	/// (if not it is subtracted)
	bool                add;
};

/*!
Structure used to pass work space from \c optimize to \c record_csum
(so that stacks do not start from zero size every time).
*/
struct struct_csum_stacks {
	/// stack of operations in the cummulative summation
	std::stack<struct struct_csum_variable>   op_stack;
	/// stack of variables to be added
	std::stack<size_t >                         add_stack;
	/// stack of variables to be subtracted
	std::stack<size_t >                         sub_stack;
};

/*!
CExpOp information that is copied to corresponding CSkipOp
*/
struct struct_cskip_info {
	/// comparision operator
	CompareOp cop;
	/// (flag & 1) is true if and only if left is a variable
	/// (flag & 2) is true if and only if right is a variable
	size_t flag;
	/// index for left comparison operand
	size_t left;
	/// index for right comparison operand
	size_t right;
	/// maximum variable index between left and right
	size_t max_left_right;
	/// set of variables to skip on true
	CppAD::vector<size_t> skip_var_true;
	/// set of variables to skip on false
	CppAD::vector<size_t> skip_var_false;
	/// set of operations to skip on true
	CppAD::vector<size_t> skip_op_true;
	/// set of operations to skip on false
	CppAD::vector<size_t> skip_op_false;
	/// size of skip_op_true
	size_t n_op_true;
	/// size of skip_op_false
	size_t n_op_false;
	/// index in the argument recording of first argument for this CSkipOp
	size_t i_arg;
};
/*!
Connection information for a user atomic function
*/
struct struct_user_info {
	/// type of connection for this atomic function
	enum_connect_type connect_type;
	/// If this is an conditional connection, this is the information
	/// of the correpsonding CondExpOp operators
	class_set_cexp_pair cexp_set;
	/// If this is a conditional connection, this is the operator
	/// index of the beginning of the atomic call sequence; i.e.,
	/// the first UserOp.
	size_t op_begin;
	/// If this is a conditional connection, this is one more than the
	///  operator index of the ending of the atomic call sequence; i.e.,
	/// the second UserOp.
	size_t op_end;
};

/*!
Shared documentation for optimization helper functions (not called).

<!-- define prototype -->
\param tape
is a vector that maps a variable index, in the old operation sequence,
to an <tt>struct_old_variable</tt> information record.
Note that the index for this vector must be greater than or equal zero and
less than <tt>tape.size()</tt>.

\li <tt>tape[i].op</tt>
is the operator in the old operation sequence
corresponding to the old variable index \c i.
Assertion: <tt>NumRes(tape[i].op) > 0</tt>.

\li <tt>tape[i].arg</tt>
for <tt>j < NumArg( tape[i].op ), tape[i].arg[j]</tt>
is the j-th the argument, in the old operation sequence,
corresponding to the old variable index \c i.
Assertion: <tt>tape[i].arg[j] < i</tt>.

\li <tt>tape[i].new_var</tt>
Suppose
<tt>i <= current, j < NumArg( tape[i].op ), and k = tape[i].arg[j]</tt>,
and \c j corresponds to a variable for operator <tt>tape[i].op</tt>.
It follows that <tt>tape[k].new_var</tt>
has alread been set to the variable in the new operation sequence
corresponding to the old variable index \c k.
This means that the \c new_var value has been set
for all the possible arguments that come before \a current.

\param current
is the index in the old operation sequence for
the variable corresponding to the result for the current operator.
Assertions:
<tt>
current < tape.size(),
NumRes( tape[current].op ) > 0.
</tt>

\param npar
is the number of parameters corresponding to this operation sequence.

\param par
is a vector of length \a npar containing the parameters
for this operation sequence; i.e.,
given a parameter index \c i, the corresponding parameter value is
<tt>par[i]</tt>.
<!-- end prototype -->
*/
template <class Base>
void prototype(
	const CppAD::vector<struct struct_old_variable>& tape           ,
	size_t                                             current        ,
	size_t                                             npar           ,
	const Base*                                        par            )
{	CPPAD_ASSERT_UNKNOWN(false); }

/*!
Check a unary operator for a complete match with a previous operator.

A complete match means that the result of the previous operator
can be used inplace of the result for current operator.

<!-- replace prototype -->
\param tape
is a vector that maps a variable index, in the old operation sequence,
to an <tt>struct_old_variable</tt> information record.
Note that the index for this vector must be greater than or equal zero and
less than <tt>tape.size()</tt>.

\li <tt>tape[i].op</tt>
is the operator in the old operation sequence
corresponding to the old variable index \c i.
Assertion: <tt>NumRes(tape[i].op) > 0</tt>.

\li <tt>tape[i].arg</tt>
for <tt>j < NumArg( tape[i].op ), tape[i].arg[j]</tt>
is the j-th the argument, in the old operation sequence,
corresponding to the old variable index \c i.
Assertion: <tt>tape[i].arg[j] < i</tt>.

\li <tt>tape[i].new_var</tt>
Suppose
<tt>i <= current, j < NumArg( tape[i].op ), and k = tape[i].arg[j]</tt>,
and \c j corresponds to a variable for operator <tt>tape[i].op</tt>.
It follows that <tt>tape[k].new_var</tt>
has alread been set to the variable in the new operation sequence
corresponding to the old variable index \c k.
This means that the \c new_var value has been set
for all the possible arguments that come before \a current.

\param current
is the index in the old operation sequence for
the variable corresponding to the result for the current operator.
Assertions:
<tt>
current < tape.size(),
NumRes( tape[current].op ) > 0.
</tt>

\param npar
is the number of parameters corresponding to this operation sequence.

\param par
is a vector of length \a npar containing the parameters
for this operation sequence; i.e.,
given a parameter index \c i, the corresponding parameter value is
<tt>par[i]</tt>.
<!-- end prototype -->

\param hash_table_var
is a vector with size CPPAD_HASH_TABLE_SIZE
that maps a hash code to the corresponding
variable index in the old operation sequence.
All the values in this table must be less than \a current.

\param code
The input value of code does not matter.
The output value of code is the hash code corresponding to
this operation in the new operation sequence.

\return
If the return value is zero,
no match was found.
If the return value is greater than zero,
it is the old operation sequence index of a variable,
that comes before current and can be used to replace the current variable.

\par Restrictions:
NumArg( tape[current].op ) == 1
*/
template <class Base>
addr_t unary_match(
	const CppAD::vector<struct struct_old_variable>& tape           ,
	size_t                                             current        ,
	size_t                                             npar           ,
	const Base*                                        par            ,
	const CppAD::vector<size_t>&                       hash_table_var ,
	unsigned short&                                    code           )
{	const addr_t* arg = tape[current].arg;
	OpCode        op  = tape[current].op;
	addr_t new_arg[1];

	// ErfOp has three arguments, but the second and third are always the
	// parameters 0 and 2 / sqrt(pi) respectively.
	CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 || op == ErfOp);
	CPPAD_ASSERT_UNKNOWN( NumRes(op) > 0  );
	CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < current );
	new_arg[0] = tape[arg[0]].new_var;
	CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < current );
	code = hash_code(
		op                  ,
		new_arg             ,
		npar                ,
		par
	);
	size_t  i_var  = hash_table_var[code];
	CPPAD_ASSERT_UNKNOWN( i_var < current );
	if( op == tape[i_var].op )
	{	size_t k = tape[i_var].arg[0];
		CPPAD_ASSERT_UNKNOWN( k < i_var );
		if (new_arg[0] == tape[k].new_var )
			return i_var;
	}
	return 0;
}

/*!
Check a binary operator for a complete match with a previous operator,

<!-- replace prototype -->
\param tape
is a vector that maps a variable index, in the old operation sequence,
to an <tt>struct_old_variable</tt> information record.
Note that the index for this vector must be greater than or equal zero and
less than <tt>tape.size()</tt>.

\li <tt>tape[i].op</tt>
is the operator in the old operation sequence
corresponding to the old variable index \c i.
Assertion: <tt>NumRes(tape[i].op) > 0</tt>.

\li <tt>tape[i].arg</tt>
for <tt>j < NumArg( tape[i].op ), tape[i].arg[j]</tt>
is the j-th the argument, in the old operation sequence,
corresponding to the old variable index \c i.
Assertion: <tt>tape[i].arg[j] < i</tt>.

\li <tt>tape[i].new_var</tt>
Suppose
<tt>i <= current, j < NumArg( tape[i].op ), and k = tape[i].arg[j]</tt>,
and \c j corresponds to a variable for operator <tt>tape[i].op</tt>.
It follows that <tt>tape[k].new_var</tt>
has alread been set to the variable in the new operation sequence
corresponding to the old variable index \c k.
This means that the \c new_var value has been set
for all the possible arguments that come before \a current.

\param current
is the index in the old operation sequence for
the variable corresponding to the result for the current operator.
Assertions:
<tt>
current < tape.size(),
NumRes( tape[current].op ) > 0.
</tt>

\param npar
is the number of parameters corresponding to this operation sequence.

\param par
is a vector of length \a npar containing the parameters
for this operation sequence; i.e.,
given a parameter index \c i, the corresponding parameter value is
<tt>par[i]</tt>.
<!-- end prototype -->

\param hash_table_var
is a vector with size CPPAD_HASH_TABLE_SIZE
that maps a hash code to the corresponding
variable index in the old operation sequence.
All the values in this table must be less than \a current.

\param code
The input value of code does not matter.
The output value of code is the hash code corresponding to
this operation in the new operation sequence.

\return
If the return value is zero,
no match was found.
If the return value is greater than zero,
it is the index of a new variable that can be used to replace the
old variable.


\par Restrictions:
The binary operator must be an addition, subtraction, multiplication, division
or power operator.  NumArg( tape[current].op ) == 1.
*/
template <class Base>
inline addr_t binary_match(
	const CppAD::vector<struct struct_old_variable>&   tape           ,
	size_t                                             current        ,
	size_t                                             npar           ,
	const Base*                                        par            ,
	const CppAD::vector<size_t>&                       hash_table_var ,
	unsigned short&                                    code           )
{	OpCode        op         = tape[current].op;
	const addr_t* arg        = tape[current].arg;
	addr_t        new_arg[2];
	bool          parameter[2];

	// initialize return value
	addr_t  match_var = 0;

	CPPAD_ASSERT_UNKNOWN( NumArg(op) == 2 );
	CPPAD_ASSERT_UNKNOWN( NumRes(op) >  0 );
	switch(op)
	{	// index op variable
		case DisOp:
		// parameter not defined for this case
		CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < current );
		new_arg[0]   = arg[0];
		new_arg[1]   = tape[arg[1]].new_var;
		break;

		// parameter op variable ----------------------------------
		case AddpvOp:
		case MulpvOp:
		case DivpvOp:
		case PowpvOp:
		case SubpvOp:
		case ZmulpvOp:
		// arg[0]
		parameter[0] = true;
		new_arg[0]   = arg[0];
		CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < npar );
		// arg[1]
		parameter[1] = false;
		new_arg[1]   = tape[arg[1]].new_var;
		CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < current );
		break;

		// variable op parameter -----------------------------------
		case DivvpOp:
		case PowvpOp:
		case SubvpOp:
		case ZmulvpOp:
		// arg[0]
		parameter[0] = false;
		new_arg[0]   = tape[arg[0]].new_var;
		CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < current );
		// arg[1]
		parameter[1] = true;
		new_arg[1]   = arg[1];
		CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < npar );
		break;

		// variable op variable -----------------------------------
		case AddvvOp:
		case MulvvOp:
		case DivvvOp:
		case PowvvOp:
		case SubvvOp:
		case ZmulvvOp:
		// arg[0]
		parameter[0] = false;
		new_arg[0]   = tape[arg[0]].new_var;
		CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < current );
		// arg[1]
		parameter[1] = false;
		new_arg[1]   = tape[arg[1]].new_var;
		CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < current );
		break;

		// must be one of the cases above
		default:
		CPPAD_ASSERT_UNKNOWN(false);
	}
	code = hash_code(
		op                  ,
		new_arg             ,
		npar                ,
		par
	);
	size_t  i_var  = hash_table_var[code];
	CPPAD_ASSERT_UNKNOWN( i_var < current );
	if( op == tape[i_var].op )
	{	bool match = true;
		if( op == DisOp )
		{	match   &= new_arg[0] == tape[i_var].arg[0];
			size_t k = tape[i_var].arg[1];
			match   &= new_arg[1] == tape[k].new_var;
		}
		else
		{	for(size_t j = 0; j < 2; j++)
			{	size_t k = tape[i_var].arg[j];
				if( parameter[j] )
				{	CPPAD_ASSERT_UNKNOWN( k < npar );
					match &= IdenticalEqualPar(
						par[ arg[j] ], par[k]
					);
				}
				else
				{	CPPAD_ASSERT_UNKNOWN( k < i_var );
					match &= (new_arg[j] == tape[k].new_var);
				}
			}
		}
		if( match )
			match_var = i_var;
	}
	if( (match_var > 0) | ( (op != AddvvOp) & (op != MulvvOp ) ) )
		return match_var;

	// check for match with argument order switched ----------------------
	CPPAD_ASSERT_UNKNOWN( op == AddvvOp || op == MulvvOp );
	std::swap(new_arg[0], new_arg[1]);
	unsigned short code_switch = hash_code(
		op                  ,
		new_arg             ,
		npar                ,
		par
	);
	i_var  = hash_table_var[code_switch];
	CPPAD_ASSERT_UNKNOWN( i_var < current );
	if( op == tape[i_var].op )
	{	bool match = true;
		size_t j;
		for(j = 0; j < 2; j++)
		{	size_t k = tape[i_var].arg[j];
			CPPAD_ASSERT_UNKNOWN( k < i_var );
			match &= (new_arg[j] == tape[k].new_var);
		}
		if( match )
			match_var = i_var;
	}
	return match_var;
}

/*!
Record an operation of the form (parameter op variable).

<!-- replace prototype -->
\param tape
is a vector that maps a variable index, in the old operation sequence,
to an <tt>struct_old_variable</tt> information record.
Note that the index for this vector must be greater than or equal zero and
less than <tt>tape.size()</tt>.

\li <tt>tape[i].op</tt>
is the operator in the old operation sequence
corresponding to the old variable index \c i.
Assertion: <tt>NumRes(tape[i].op) > 0</tt>.

\li <tt>tape[i].arg</tt>
for <tt>j < NumArg( tape[i].op ), tape[i].arg[j]</tt>
is the j-th the argument, in the old operation sequence,
corresponding to the old variable index \c i.
Assertion: <tt>tape[i].arg[j] < i</tt>.

\li <tt>tape[i].new_var</tt>
Suppose
<tt>i <= current, j < NumArg( tape[i].op ), and k = tape[i].arg[j]</tt>,
and \c j corresponds to a variable for operator <tt>tape[i].op</tt>.
It follows that <tt>tape[k].new_var</tt>
has alread been set to the variable in the new operation sequence
corresponding to the old variable index \c k.
This means that the \c new_var value has been set
for all the possible arguments that come before \a current.

\param current
is the index in the old operation sequence for
the variable corresponding to the result for the current operator.
Assertions:
<tt>
current < tape.size(),
NumRes( tape[current].op ) > 0.
</tt>

\param npar
is the number of parameters corresponding to this operation sequence.

\param par
is a vector of length \a npar containing the parameters
for this operation sequence; i.e.,
given a parameter index \c i, the corresponding parameter value is
<tt>par[i]</tt>.
<!-- end prototype -->

\param rec
is the object that will record the operations.

\param op
is the operator that we are recording which must be one of the following:
AddpvOp, DivpvOp, MulpvOp, PowpvOp, SubpvOp, ZmulpvOp.

\param arg
is the vector of arguments for this operator.

\return
the result is the operaiton and variable index corresponding to the current
operation in the new operation sequence.
*/
template <class Base>
struct_size_pair record_pv(
	const CppAD::vector<struct struct_old_variable>& tape           ,
	size_t                                             current        ,
	size_t                                             npar           ,
	const Base*                                        par            ,
	recorder<Base>*                                    rec            ,
	OpCode                                             op             ,
	const addr_t*                                      arg            )
{
# ifndef NDEBUG
	switch(op)
	{	case AddpvOp:
		case DivpvOp:
		case MulpvOp:
		case PowpvOp:
		case SubpvOp:
		case ZmulpvOp:
		break;

		default:
		CPPAD_ASSERT_UNKNOWN(false);
	}
# endif
	CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < npar    );
	CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < current );
	addr_t new_arg[2];
	new_arg[0]   = rec->PutPar( par[arg[0]] );
	new_arg[1]   = tape[ arg[1] ].new_var;
	rec->PutArg( new_arg[0], new_arg[1] );

	struct_size_pair ret;
	ret.i_op  = rec->num_op_rec();
	ret.i_var = rec->PutOp(op);
	CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < ret.i_var );
	return ret;
}


/*!
Record an operation of the form (variable op parameter).

<!-- replace prototype -->
\param tape
is a vector that maps a variable index, in the old operation sequence,
to an <tt>struct_old_variable</tt> information record.
Note that the index for this vector must be greater than or equal zero and
less than <tt>tape.size()</tt>.

\li <tt>tape[i].op</tt>
is the operator in the old operation sequence
corresponding to the old variable index \c i.
Assertion: <tt>NumRes(tape[i].op) > 0</tt>.

\li <tt>tape[i].arg</tt>
for <tt>j < NumArg( tape[i].op ), tape[i].arg[j]</tt>
is the j-th the argument, in the old operation sequence,
corresponding to the old variable index \c i.
Assertion: <tt>tape[i].arg[j] < i</tt>.

\li <tt>tape[i].new_var</tt>
Suppose
<tt>i <= current, j < NumArg( tape[i].op ), and k = tape[i].arg[j]</tt>,
and \c j corresponds to a variable for operator <tt>tape[i].op</tt>.
It follows that <tt>tape[k].new_var</tt>
has alread been set to the variable in the new operation sequence
corresponding to the old variable index \c k.
This means that the \c new_var value has been set
for all the possible arguments that come before \a current.

\param current
is the index in the old operation sequence for
the variable corresponding to the result for the current operator.
Assertions:
<tt>
current < tape.size(),
NumRes( tape[current].op ) > 0.
</tt>

\param npar
is the number of parameters corresponding to this operation sequence.

\param par
is a vector of length \a npar containing the parameters
for this operation sequence; i.e.,
given a parameter index \c i, the corresponding parameter value is
<tt>par[i]</tt>.
<!-- end prototype -->

\param rec
is the object that will record the operations.

\param op
is the operator that we are recording which must be one of the following:
DivvpOp, PowvpOp, SubvpOp, ZmulvpOp.

\param arg
is the vector of arguments for this operator.

\return
the result operation and variable index corresponding to the current
operation in the new operation sequence.
*/
template <class Base>
struct_size_pair record_vp(
	const CppAD::vector<struct struct_old_variable>& tape           ,
	size_t                                             current        ,
	size_t                                             npar           ,
	const Base*                                        par            ,
	recorder<Base>*                                    rec            ,
	OpCode                                             op             ,
	const addr_t*                                      arg            )
{
# ifndef NDEBUG
	switch(op)
	{	case DivvpOp:
		case PowvpOp:
		case SubvpOp:
		case ZmulvpOp:
		break;

		default:
		CPPAD_ASSERT_UNKNOWN(false);
	}
# endif
	CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < current );
	CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < npar    );
	addr_t new_arg[2];
	new_arg[0]   = tape[ arg[0] ].new_var;
	new_arg[1]   = rec->PutPar( par[arg[1]] );
	rec->PutArg( new_arg[0], new_arg[1] );

	struct_size_pair ret;
	ret.i_op  = rec->num_op_rec();
	ret.i_var = rec->PutOp(op);
	CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < ret.i_var );
	return ret;
}

/*!
Record an operation of the form (variable op variable).

<!-- replace prototype -->
\param tape
is a vector that maps a variable index, in the old operation sequence,
to an <tt>struct_old_variable</tt> information record.
Note that the index for this vector must be greater than or equal zero and
less than <tt>tape.size()</tt>.

\li <tt>tape[i].op</tt>
is the operator in the old operation sequence
corresponding to the old variable index \c i.
Assertion: <tt>NumRes(tape[i].op) > 0</tt>.

\li <tt>tape[i].arg</tt>
for <tt>j < NumArg( tape[i].op ), tape[i].arg[j]</tt>
is the j-th the argument, in the old operation sequence,
corresponding to the old variable index \c i.
Assertion: <tt>tape[i].arg[j] < i</tt>.

\li <tt>tape[i].new_var</tt>
Suppose
<tt>i <= current, j < NumArg( tape[i].op ), and k = tape[i].arg[j]</tt>,
and \c j corresponds to a variable for operator <tt>tape[i].op</tt>.
It follows that <tt>tape[k].new_var</tt>
has alread been set to the variable in the new operation sequence
corresponding to the old variable index \c k.
This means that the \c new_var value has been set
for all the possible arguments that come before \a current.

\param current
is the index in the old operation sequence for
the variable corresponding to the result for the current operator.
Assertions:
<tt>
current < tape.size(),
NumRes( tape[current].op ) > 0.
</tt>

\param npar
is the number of parameters corresponding to this operation sequence.

\param par
is a vector of length \a npar containing the parameters
for this operation sequence; i.e.,
given a parameter index \c i, the corresponding parameter value is
<tt>par[i]</tt>.
<!-- end prototype -->

\param rec
is the object that will record the operations.

\param op
is the operator that we are recording which must be one of the following:
AddvvOp, DivvvOp, MulvvOp, PowvvOp, SubvvOp, ZmulvvOp.

\param arg
is the vector of arguments for this operator.

\return
the result is the operation and variable index corresponding to the current
operation in the new operation sequence.
*/
template <class Base>
struct_size_pair record_vv(
	const CppAD::vector<struct struct_old_variable>& tape           ,
	size_t                                             current        ,
	size_t                                             npar           ,
	const Base*                                        par            ,
	recorder<Base>*                                    rec            ,
	OpCode                                             op             ,
	const addr_t*                                      arg            )
{
# ifndef NDEBUG
	switch(op)
	{	case AddvvOp:
		case DivvvOp:
		case MulvvOp:
		case PowvvOp:
		case SubvvOp:
		case ZmulvvOp:
		break;

		default:
		CPPAD_ASSERT_UNKNOWN(false);
	}
# endif
	CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < current );
	CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < current );
	addr_t new_arg[2];
	new_arg[0]   = tape[ arg[0] ].new_var;
	new_arg[1]   = tape[ arg[1] ].new_var;
	rec->PutArg( new_arg[0], new_arg[1] );

	struct_size_pair ret;
	ret.i_op  = rec->num_op_rec();
	ret.i_var = rec->PutOp(op);
	CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < ret.i_var );
	CPPAD_ASSERT_UNKNOWN( size_t(new_arg[1]) < ret.i_var );
	return ret;
}

// ==========================================================================

/*!
Recording a cummulative cummulative summation starting at its highest parrent.

<!-- replace prototype -->
\param tape
is a vector that maps a variable index, in the old operation sequence,
to an <tt>struct_old_variable</tt> information record.
Note that the index for this vector must be greater than or equal zero and
less than <tt>tape.size()</tt>.

\li <tt>tape[i].op</tt>
is the operator in the old operation sequence
corresponding to the old variable index \c i.
Assertion: <tt>NumRes(tape[i].op) > 0</tt>.

\li <tt>tape[i].arg</tt>
for <tt>j < NumArg( tape[i].op ), tape[i].arg[j]</tt>
is the j-th the argument, in the old operation sequence,
corresponding to the old variable index \c i.
Assertion: <tt>tape[i].arg[j] < i</tt>.

\li <tt>tape[i].new_var</tt>
Suppose
<tt>i <= current, j < NumArg( tape[i].op ), and k = tape[i].arg[j]</tt>,
and \c j corresponds to a variable for operator <tt>tape[i].op</tt>.
It follows that <tt>tape[k].new_var</tt>
has alread been set to the variable in the new operation sequence
corresponding to the old variable index \c k.
This means that the \c new_var value has been set
for all the possible arguments that come before \a current.

\param current
is the index in the old operation sequence for
the variable corresponding to the result for the current operator.
Assertions:
<tt>
current < tape.size(),
NumRes( tape[current].op ) > 0.
</tt>

\param npar
is the number of parameters corresponding to this operation sequence.

\param par
is a vector of length \a npar containing the parameters
for this operation sequence; i.e.,
given a parameter index \c i, the corresponding parameter value is
<tt>par[i]</tt>.
<!-- end prototype -->

\param rec
is the object that will record the operations.

\param work
Is used for computation. On input and output,
<tt>work.op_stack.empty()</tt>,
<tt>work.add_stack.empty()</tt>, and
<tt>work.sub_stack.empty()</tt>,
are all true true.
These stacks are passed in so that elements can be allocated once
and then the elements can be reused with calls to \c record_csum.

\par Exception
<tt>tape[i].new_var</tt>
is not yet defined for any node \c i that is \c csum_connected
to the \a current node
(or that is \c sum_connected to a node that is \c csum_connected).
For example; suppose that index \c j corresponds to a variable
in the current operator,
<tt>i = tape[current].arg[j]</tt>,
and
<tt>tape[arg[j]].connect_type == csum_connected</tt>.
It then follows that
<tt>tape[i].new_var == tape.size()</tt>.

\par Restriction:
\li <tt>tape[current].op</tt>
must be one of <tt>AddpvOp, AddvvOp, SubpvOp, SubvpOp, SubvvOp</tt>.

\li <tt>tape[current].connect_type</tt> must be \c yes_connected.

\li <tt>tape[j].connect_type == csum_connected</tt> for some index
j that is a variable operand for the current operation.
*/


template <class Base>
struct_size_pair record_csum(
	const CppAD::vector<struct struct_old_variable>& tape           ,
	size_t                                             current        ,
	size_t                                             npar           ,
	const Base*                                        par            ,
	recorder<Base>*                                    rec            ,
	struct_csum_stacks&                              work           )
{

	CPPAD_ASSERT_UNKNOWN( work.op_stack.empty() );
	CPPAD_ASSERT_UNKNOWN( work.add_stack.empty() );
	CPPAD_ASSERT_UNKNOWN( work.sub_stack.empty() );
	CPPAD_ASSERT_UNKNOWN( tape[current].connect_type == yes_connected );

	size_t                        i;
	OpCode                        op;
	const addr_t*                 arg;
	bool                          add;
	struct struct_csum_variable var;

	var.op  = tape[current].op;
	var.arg = tape[current].arg;
	var.add = true;
	work.op_stack.push( var );
	Base sum_par(0);

# ifndef NDEBUG
	bool ok = false;
	if( var.op == SubvpOp )
		ok = tape[ tape[current].arg[0] ].connect_type == csum_connected;
	if( var.op == AddpvOp || var.op == SubpvOp )
		ok = tape[ tape[current].arg[1] ].connect_type == csum_connected;
	if( var.op == AddvvOp || var.op == SubvvOp )
	{	ok  = tape[ tape[current].arg[0] ].connect_type == csum_connected;
		ok |= tape[ tape[current].arg[1] ].connect_type == csum_connected;
	}
	CPPAD_ASSERT_UNKNOWN( ok );
# endif
	while( ! work.op_stack.empty() )
	{	var     = work.op_stack.top();
		work.op_stack.pop();
		op      = var.op;
		arg     = var.arg;
		add     = var.add;
		// process first argument to this operator
		switch(op)
		{	case AddpvOp:
			case SubpvOp:
			CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < npar );
			if( add )
				sum_par += par[arg[0]];
			else	sum_par -= par[arg[0]];
			break;

			case AddvvOp:
			case SubvpOp:
			case SubvvOp:
			if( tape[arg[0]].connect_type == csum_connected )
			{	CPPAD_ASSERT_UNKNOWN(
					size_t(tape[arg[0]].new_var) == tape.size()
				);
				var.op  = tape[arg[0]].op;
				var.arg = tape[arg[0]].arg;
				var.add = add;
				work.op_stack.push( var );
			}
			else if( add )
				work.add_stack.push(arg[0]);
			else	work.sub_stack.push(arg[0]);
			break;

			default:
			CPPAD_ASSERT_UNKNOWN(false);
		}
		// process second argument to this operator
		switch(op)
		{
			case SubvpOp:
			CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < npar );
			if( add )
				sum_par -= par[arg[1]];
			else	sum_par += par[arg[1]];
			break;

			case SubvvOp:
			case SubpvOp:
			add = ! add;

			case AddvvOp:
			case AddpvOp:
			if( tape[arg[1]].connect_type == csum_connected )
			{	CPPAD_ASSERT_UNKNOWN(
					size_t(tape[arg[1]].new_var) == tape.size()
				);
				var.op   = tape[arg[1]].op;
				var.arg  = tape[arg[1]].arg;
				var.add  = add;
				work.op_stack.push( var );
			}
			else if( add )
				work.add_stack.push(arg[1]);
			else	work.sub_stack.push(arg[1]);
			break;

			default:
			CPPAD_ASSERT_UNKNOWN(false);
		}
	}
	// number of variables in this cummulative sum operator
	size_t n_add = work.add_stack.size();
	size_t n_sub = work.sub_stack.size();
	size_t old_arg, new_arg;
	rec->PutArg(n_add);                // arg[0]
	rec->PutArg(n_sub);                // arg[1]
	new_arg = rec->PutPar( sum_par );
	rec->PutArg(new_arg);              // arg[2]
	for(i = 0; i < n_add; i++)
	{	CPPAD_ASSERT_UNKNOWN( ! work.add_stack.empty() );
		old_arg = work.add_stack.top();
		new_arg = tape[old_arg].new_var;
		CPPAD_ASSERT_UNKNOWN( new_arg < tape.size() );
		rec->PutArg(new_arg);      // arg[3+i]
		work.add_stack.pop();
	}
	for(i = 0; i < n_sub; i++)
	{	CPPAD_ASSERT_UNKNOWN( ! work.sub_stack.empty() );
		old_arg = work.sub_stack.top();
		new_arg = tape[old_arg].new_var;
		CPPAD_ASSERT_UNKNOWN( new_arg < tape.size() );
		rec->PutArg(new_arg);      // arg[3 + arg[0] + i]
		work.sub_stack.pop();
	}
	rec->PutArg(n_add + n_sub);        // arg[3 + arg[0] + arg[1]]


	struct_size_pair ret;
	ret.i_op  = rec->num_op_rec();
	ret.i_var = rec->PutOp(CSumOp);
	CPPAD_ASSERT_UNKNOWN( new_arg < ret.i_var );
	return ret;
}
// ==========================================================================
/*!
Convert a player object to an optimized recorder object

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< \a Base > and computations by this routine are done using type
\a Base.

\param options
The possible values for this string are:
"", "no_conditional_skip".
If it is "no_conditional_skip", then no conditional skip operations
will be generated.

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
	// nan with type Base
	Base base_nan = Base( std::numeric_limits<double>::quiet_NaN() );

	// temporary indices
	size_t i, j, k;

	// check options
	bool conditional_skip =
		options.find("no_conditional_skip", 0) == std::string::npos;

	// temporary variables
	OpCode        op;   // current operator
	const addr_t* arg;  // operator arguments
	size_t        i_var;  // index of first result for current operator

	// range and domain dimensions for F
	size_t m = dep_taddr.size();

	// number of variables in the player
	const size_t num_var = play->num_var_rec();

# ifndef NDEBUG
	// number of parameters in the player
	const size_t num_par = play->num_par_rec();
# endif

	// number of  VecAD indices
	size_t num_vecad_ind   = play->num_vec_ind_rec();

	// number of VecAD vectors
	size_t num_vecad_vec   = play->num_vecad_vec_rec();

	// -------------------------------------------------------------
	// data structure that maps variable index in original operation
	// sequence to corresponding operator information
	CppAD::vector<struct struct_old_variable> tape(num_var);

	// if tape[i].connect_type == exp_connected, cexp_set[i] is the
	// corresponding information for the conditional connection.
	CppAD::vector<class_set_cexp_pair> cexp_vec_set;
	if( conditional_skip )
		cexp_vec_set.resize(num_var);
	// -------------------------------------------------------------
	// Determine how each variable is connected to the dependent variables

	// initialize all variables has having no connections
	for(i = 0; i < num_var; i++)
		tape[i].connect_type = not_connected;

	for(j = 0; j < m; j++)
	{	// mark dependent variables as having one or more connections
		tape[ dep_taddr[j] ].connect_type = yes_connected;
	}

	// vecad_connect contains a value for each VecAD object.
	// vecad maps a VecAD index (which corresponds to the beginning of the
	// VecAD object) to the vecad_connect falg for the VecAD object.
	CppAD::vector<enum_connect_type>   vecad_connect(num_vecad_vec);
	CppAD::vector<size_t> vecad(num_vecad_ind);
	j = 0;
	for(i = 0; i < num_vecad_vec; i++)
	{	vecad_connect[i] = not_connected;
		// length of this VecAD
		size_t length = play->GetVecInd(j);
		// set to proper index for this VecAD
		vecad[j] = i;
		for(k = 1; k <= length; k++)
			vecad[j+k] = num_vecad_vec; // invalid index
		// start of next VecAD
		j       += length + 1;
	}
	CPPAD_ASSERT_UNKNOWN( j == num_vecad_ind );

	// work space used by UserOp.
	typedef std::set<size_t> size_set;
	//
	vector<size_set> user_r_set;   // set sparsity pattern for result
	vector<size_set> user_s_set;   // set sparisty pattern for argument
	//
	vector<bool>     user_r_bool;  // bool sparsity pattern for result
	vector<bool>     user_s_bool;  // bool sparisty pattern for argument
	//
	vectorBool       user_r_pack;  // pack sparsity pattern for result
	vectorBool       user_s_pack;  // pack sparisty pattern for argument
	//
	size_t user_q     = 0;       // column dimension for sparsity patterns
	size_t user_index = 0;       // indentifier for this user_atomic operation
	size_t user_id    = 0;       // user identifier for this call to operator
	size_t user_i     = 0;       // index in result vector
	size_t user_j     = 0;       // index in argument vector
	size_t user_m     = 0;       // size of result vector
	size_t user_n     = 0;       // size of arugment vector
	//
	atomic_base<Base>* user_atom = CPPAD_NULL; // current user atomic function
	bool               user_pack = false;      // sparsity pattern type is pack
	bool               user_bool = false;      // sparsity pattern type is bool
	bool               user_set  = false;      // sparsity pattern type is set

	// next expected operator in a UserOp sequence
	enum { user_start, user_arg, user_ret, user_end } user_state;

	// During reverse mode, compute type of connection for each call to
	// a user atomic function.
	CppAD::vector<struct_user_info>    user_info;
	size_t                             user_curr = 0;

	/// During reverse mode, information for each CSkip operation
	CppAD::vector<struct_cskip_info>   cskip_info;

	// Initialize a reverse mode sweep through the operation sequence
	size_t i_op;
	play->reverse_start(op, arg, i_op, i_var);
	CPPAD_ASSERT_UNKNOWN( op == EndOp );
	size_t mask;
	user_state = user_end;
	while(op != BeginOp)
	{	// next op
		play->reverse_next(op, arg, i_op, i_var);

		// Store the operator corresponding to each variable
		if( NumRes(op) > 0 )
		{	tape[i_var].op = op;
			tape[i_var].arg = arg;
		}
# ifndef NDEBUG
		if( i_op <= n )
		{	CPPAD_ASSERT_UNKNOWN((op == InvOp) | (op == BeginOp));
		}
		else	CPPAD_ASSERT_UNKNOWN((op != InvOp) & (op != BeginOp));
# endif
		enum_connect_type connect_type      = tape[i_var].connect_type;
		class_set_cexp_pair* cexp_set = CPPAD_NULL;
		if( conditional_skip )
			cexp_set = &cexp_vec_set[i_var];
		switch( op )
		{
			// One variable corresponding to arg[0]
			case AbsOp:
			case AcosOp:
			case AcoshOp:
			case AsinOp:
			case AsinhOp:
			case AtanOp:
			case AtanhOp:
			case CosOp:
			case CoshOp:
			case DivvpOp:
			case ErfOp:
			case ExpOp:
			case Expm1Op:
			case LogOp:
			case Log1pOp:
			case PowvpOp:
			case SignOp:
			case SinOp:
			case SinhOp:
			case SqrtOp:
			case TanOp:
			case TanhOp:
			case ZmulvpOp:
			switch( connect_type )
			{	case not_connected:
				break;

				case yes_connected:
				case sum_connected:
				case csum_connected:
				tape[arg[0]].connect_type = yes_connected;
				break;

				case cexp_connected:
				CPPAD_ASSERT_UNKNOWN( conditional_skip )
				if( tape[arg[0]].connect_type == not_connected )
				{	tape[arg[0]].connect_type = cexp_connected;
					cexp_vec_set[arg[0]]     = *cexp_set;
				}
				else if( tape[arg[0]].connect_type == cexp_connected )
				{	cexp_vec_set[arg[0]].intersection(*cexp_set);
					if( cexp_vec_set[arg[0]].empty() )
						tape[arg[0]].connect_type = yes_connected;
				}
				else	tape[arg[0]].connect_type = yes_connected;
				break;

				default:
				CPPAD_ASSERT_UNKNOWN(false);
			}
			break; // --------------------------------------------

			// One variable corresponding to arg[1]
			case DisOp:
			case DivpvOp:
			case MulpvOp:
			case PowpvOp:
			case ZmulpvOp:
			switch( connect_type )
			{	case not_connected:
				break;

				case yes_connected:
				case sum_connected:
				case csum_connected:
				tape[arg[1]].connect_type = yes_connected;
				break;

				case cexp_connected:
				CPPAD_ASSERT_UNKNOWN( conditional_skip )
				if( tape[arg[1]].connect_type == not_connected )
				{	tape[arg[1]].connect_type = cexp_connected;
					cexp_vec_set[arg[1]]     = *cexp_set;
				}
				else if( tape[arg[1]].connect_type == cexp_connected )
				{	cexp_vec_set[arg[1]].intersection(*cexp_set);
					if( cexp_vec_set[arg[1]].empty() )
						tape[arg[1]].connect_type = yes_connected;
				}
				else	tape[arg[1]].connect_type = yes_connected;
				break;

				default:
				CPPAD_ASSERT_UNKNOWN(false);
			}
			break; // --------------------------------------------

			// Special case for SubvpOp
			case SubvpOp:
			switch( connect_type )
			{	case not_connected:
				break;

				case yes_connected:
				case sum_connected:
				case csum_connected:
				if( tape[arg[0]].connect_type == not_connected )
					tape[arg[0]].connect_type = sum_connected;
				else	tape[arg[0]].connect_type = yes_connected;
				break;

				case cexp_connected:
				CPPAD_ASSERT_UNKNOWN( conditional_skip )
				if( tape[arg[0]].connect_type == not_connected )
				{	tape[arg[0]].connect_type = cexp_connected;
					cexp_vec_set[arg[0]]     = *cexp_set;
				}
				else if( tape[arg[0]].connect_type == cexp_connected )
				{	cexp_vec_set[arg[0]].intersection(*cexp_set);
					if( cexp_vec_set[arg[0]].empty() )
						tape[arg[0]].connect_type = yes_connected;
				}
				else	tape[arg[0]].connect_type = yes_connected;
				break;

				default:
				CPPAD_ASSERT_UNKNOWN(false);
			}
			if( connect_type == sum_connected )
			{	// convert sum to csum connection for this variable
				tape[i_var].connect_type = connect_type = csum_connected;
			}
			break; // --------------------------------------------

			// Special case for AddpvOp and SubpvOp
			case AddpvOp:
			case SubpvOp:
			switch( connect_type )
			{	case not_connected:
				break;

				case yes_connected:
				case sum_connected:
				case csum_connected:
				if( tape[arg[1]].connect_type == not_connected )
					tape[arg[1]].connect_type = sum_connected;
				else	tape[arg[1]].connect_type = yes_connected;
				break;

				case cexp_connected:
				CPPAD_ASSERT_UNKNOWN( conditional_skip )
				if( tape[arg[1]].connect_type == not_connected )
				{	tape[arg[1]].connect_type = cexp_connected;
					cexp_vec_set[arg[1]]     = *cexp_set;
				}
				else if( tape[arg[1]].connect_type == cexp_connected )
				{	cexp_vec_set[arg[1]].intersection(*cexp_set);
					if( cexp_vec_set[arg[1]].empty() )
						tape[arg[1]].connect_type = yes_connected;
				}
				else	tape[arg[1]].connect_type = yes_connected;
				break;

				default:
				CPPAD_ASSERT_UNKNOWN(false);
			}
			if( connect_type == sum_connected )
			{	// convert sum to csum connection for this variable
				tape[i_var].connect_type = connect_type = csum_connected;
			}
			break; // --------------------------------------------


			// Special case for AddvvOp and SubvvOp
			case AddvvOp:
			case SubvvOp:
			for(i = 0; i < 2; i++) switch( connect_type )
			{	case not_connected:
				break;

				case yes_connected:
				case sum_connected:
				case csum_connected:
				if( tape[arg[i]].connect_type == not_connected )
					tape[arg[i]].connect_type = sum_connected;
				else	tape[arg[i]].connect_type = yes_connected;
				break;

				case cexp_connected:
				CPPAD_ASSERT_UNKNOWN( conditional_skip )
				if( tape[arg[i]].connect_type == not_connected )
				{	tape[arg[i]].connect_type = cexp_connected;
					cexp_vec_set[arg[i]]     = *cexp_set;
				}
				else if( tape[arg[i]].connect_type == cexp_connected )
				{	cexp_vec_set[arg[i]].intersection(*cexp_set);
					if( cexp_vec_set[arg[i]].empty() )
						tape[arg[i]].connect_type = yes_connected;
				}
				else	tape[arg[i]].connect_type = yes_connected;
				break;

				default:
				CPPAD_ASSERT_UNKNOWN(false);
			}
			if( connect_type == sum_connected )
			{	// convert sum to csum connection for this variable
				tape[i_var].connect_type = connect_type = csum_connected;
			}
			break; // --------------------------------------------

			// Other binary operators
			// where operands are arg[0], arg[1]
			case DivvvOp:
			case MulvvOp:
			case PowvvOp:
			case ZmulvvOp:
			for(i = 0; i < 2; i++) switch( connect_type )
			{	case not_connected:
				break;

				case yes_connected:
				case sum_connected:
				case csum_connected:
				tape[arg[i]].connect_type = yes_connected;
				break;

				case cexp_connected:
				CPPAD_ASSERT_UNKNOWN( conditional_skip )
				if( tape[arg[i]].connect_type == not_connected )
				{	tape[arg[i]].connect_type = cexp_connected;
					cexp_vec_set[arg[i]]     = *cexp_set;
				}
				else if( tape[arg[i]].connect_type == cexp_connected )
				{	cexp_vec_set[arg[i]].intersection(*cexp_set);
					if( cexp_vec_set[arg[i]].empty() )
						tape[arg[i]].connect_type = yes_connected;
				}
				else	tape[arg[i]].connect_type = yes_connected;
				break;

				default:
				CPPAD_ASSERT_UNKNOWN(false);
			}
			break; // --------------------------------------------

			// Conditional expression operators
			case CExpOp:
			CPPAD_ASSERT_UNKNOWN( NumArg(CExpOp) == 6 );
			if( connect_type != not_connected )
			{	struct_cskip_info info;
				info.cop        = CompareOp( arg[0] );
				info.flag       = arg[1];
				info.left       = arg[2];
				info.right      = arg[3];
				info.n_op_true  = 0;
				info.n_op_false = 0;
				info.i_arg      = 0; // case where no CSkipOp for this CExpOp
				//
				size_t index    = 0;
				if( arg[1] & 1 )
				{	index = std::max(index, info.left);
					tape[info.left].connect_type = yes_connected;
				}
				if( arg[1] & 2 )
				{	index = std::max(index, info.right);
					tape[info.right].connect_type = yes_connected;
				}
				CPPAD_ASSERT_UNKNOWN( index > 0 );
				info.max_left_right = index;
				//
				index = cskip_info.size();
				cskip_info.push_back(info);
				//
				if( arg[1] & 4 )
				{	if( conditional_skip &&
						tape[arg[4]].connect_type == not_connected )
					{	tape[arg[4]].connect_type = cexp_connected;
						cexp_vec_set[arg[4]]     = *cexp_set;
						cexp_vec_set[arg[4]].insert(
							class_cexp_pair(true, index)
						);
					}
					else
					{	// if arg[4] is cexp_connected, it could be
						// connected for both the true and false case
						// 2DO: if previously cexp_connected
						// and the true/false sense is the same, should
						// keep this conditional connnection.
						if(conditional_skip)
							cexp_vec_set[arg[4]].clear();
						tape[arg[4]].connect_type = yes_connected;
					}
				}
				if( arg[1] & 8 )
				{	if( conditional_skip &&
						tape[arg[5]].connect_type == not_connected )
					{	tape[arg[5]].connect_type = cexp_connected;
						cexp_vec_set[arg[5]]     = *cexp_set;
						cexp_vec_set[arg[5]].insert(
							class_cexp_pair(false, index)
						);
					}
					else
					{	if(conditional_skip)
							cexp_vec_set[arg[5]].clear();
						tape[arg[5]].connect_type = yes_connected;
					}
				}
			}
			break;  // --------------------------------------------

			// Operations where there is nothing to do
			case EndOp:
			case ParOp:
			case PriOp:
			break;  // --------------------------------------------

			// Operators that never get removed
			case BeginOp:
			case InvOp:
			tape[i_var].connect_type = yes_connected;
			break;

			// Compare operators never get removed -----------------
			case LepvOp:
			case LtpvOp:
			case EqpvOp:
			case NepvOp:
			tape[arg[1]].connect_type = yes_connected;
			break;

			case LevpOp:
			case LtvpOp:
			tape[arg[0]].connect_type = yes_connected;
			break;

			case LevvOp:
			case LtvvOp:
			case EqvvOp:
			case NevvOp:
			tape[arg[0]].connect_type = yes_connected;
			tape[arg[1]].connect_type = yes_connected;
			break;

			// Load using a parameter index ----------------------
			case LdpOp:
			if( tape[i_var].connect_type != not_connected )
			{
				i                = vecad[ arg[0] - 1 ];
				vecad_connect[i] = yes_connected;
			}
			break; // --------------------------------------------

			// Load using a variable index
			case LdvOp:
			if( tape[i_var].connect_type != not_connected )
			{
				i                    = vecad[ arg[0] - 1 ];
				vecad_connect[i]     = yes_connected;
				tape[arg[1]].connect_type = yes_connected;
			}
			break; // --------------------------------------------

			// Store a variable using a parameter index
			case StpvOp:
			i = vecad[ arg[0] - 1 ];
			if( vecad_connect[i] != not_connected )
				tape[arg[2]].connect_type = yes_connected;
			break; // --------------------------------------------

			// Store a variable using a variable index
			case StvvOp:
			i = vecad[ arg[0] - 1 ];
			if( vecad_connect[i] )
			{	tape[arg[1]].connect_type = yes_connected;
				tape[arg[2]].connect_type = yes_connected;
			}
			break;
			// ============================================================
			case UserOp:
			// start or end atomic operation sequence
			CPPAD_ASSERT_UNKNOWN( NumRes( UserOp ) == 0 );
			CPPAD_ASSERT_UNKNOWN( NumArg( UserOp ) == 4 );
			if( user_state == user_end )
			{	user_index = arg[0];
				user_id    = arg[1];
				user_n     = arg[2];
				user_m     = arg[3];
				user_q     = 1;
				user_atom  = atomic_base<Base>::class_object(user_index);
				if( user_atom == CPPAD_NULL )
				{	std::string msg =
						atomic_base<Base>::class_name(user_index)
						+ ": atomic_base function has been deleted";
					CPPAD_ASSERT_KNOWN(false, msg.c_str() );
				}
				user_pack  = user_atom->sparsity() ==
							atomic_base<Base>::pack_sparsity_enum;
				user_bool  = user_atom->sparsity() ==
							atomic_base<Base>::bool_sparsity_enum;
				user_set   = user_atom->sparsity() ==
							atomic_base<Base>::set_sparsity_enum;
				CPPAD_ASSERT_UNKNOWN( user_pack || user_bool || user_set );

				user_set   = user_atom->sparsity() ==
					atomic_base<Base>::set_sparsity_enum;
				//
				// Note user_q is 1, but use it for clarity of code
				if( user_pack )
				{	if( user_r_pack.size() != user_m * user_q )
						user_r_pack.resize( user_m * user_q );
					if( user_s_pack.size() != user_n * user_q )
						user_s_pack.resize( user_n * user_q );
					for(i = 0; i < user_m; i++)
						for(j = 0; j < user_q; j++)
							user_r_pack[ i * user_q + j] = false;
				}
				if( user_bool )
				{	if( user_r_bool.size() != user_m * user_q )
						user_r_bool.resize( user_m * user_q );
					if( user_s_bool.size() != user_n * user_q )
						user_s_bool.resize( user_n * user_q );
					for(i = 0; i < user_m; i++)
						for(j = 0; j < user_q; j++)
							user_r_bool[ i * user_q + j] = false;
				}
				if( user_set )
				{	if(user_s_set.size() != user_n )
						user_s_set.resize(user_n);
					if(user_r_set.size() != user_m )
						user_r_set.resize(user_m);
						for(i = 0; i < user_m; i++)
							user_r_set[i].clear();
				}
				//
				user_j     = user_n;
				user_i     = user_m;
				user_state = user_ret;
				//
				struct_user_info info;
				info.connect_type = not_connected;
				info.op_end       = i_op + 1;
				user_info.push_back(info);

			}
			else
			{	CPPAD_ASSERT_UNKNOWN( user_state == user_start );
				CPPAD_ASSERT_UNKNOWN( user_index == size_t(arg[0]) );
				CPPAD_ASSERT_UNKNOWN( user_id    == size_t(arg[1]) );
				CPPAD_ASSERT_UNKNOWN( user_n     == size_t(arg[2]) );
				CPPAD_ASSERT_UNKNOWN( user_m     == size_t(arg[3]) );
				user_state = user_end;
				//
				CPPAD_ASSERT_UNKNOWN( user_curr + 1 == user_info.size() );
				user_info[user_curr].op_begin = i_op;
				user_curr                     = user_info.size();
               }
			break;

			case UsrapOp:
			// parameter argument in an atomic operation sequence
			CPPAD_ASSERT_UNKNOWN( user_state == user_arg );
			CPPAD_ASSERT_UNKNOWN( 0 < user_j && user_j <= user_n );
			CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 );
			CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_par );
			--user_j;
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
			if( user_set )
			{	if( ! user_s_set[user_j].empty() )
					tape[arg[0]].connect_type =
						user_info[user_curr].connect_type;
			}
			if( user_bool )
			{	if( user_s_bool[user_j] )
					tape[arg[0]].connect_type =
						user_info[user_curr].connect_type;
			}
			if( user_pack )
			{	if( user_s_pack[user_j] )
					tape[arg[0]].connect_type =
						user_info[user_curr].connect_type;
			}
			if( user_j == 0 )
				user_state = user_start;
			break;

			case UsrrvOp:
			// variable result in an atomic operation sequence
			CPPAD_ASSERT_UNKNOWN( user_state == user_ret );
			CPPAD_ASSERT_UNKNOWN( 0 < user_i && user_i <= user_m );
			--user_i;
			switch( connect_type )
			{	case not_connected:
				break;

				case yes_connected:
				case sum_connected:
				case csum_connected:
				user_info[user_curr].connect_type = yes_connected;
				if( user_set )
					user_r_set[user_i].insert(0);
				if( user_bool )
					user_r_bool[user_i] = true;
				if( user_pack )
					user_r_pack[user_i] = true;
				break;

				case cexp_connected:
				CPPAD_ASSERT_UNKNOWN( conditional_skip );
				if( user_info[user_curr].connect_type == not_connected )
				{	user_info[user_curr].connect_type  = connect_type;
					user_info[user_curr].cexp_set      = *cexp_set;
				}
				else if(user_info[user_curr].connect_type==cexp_connected)
				{	user_info[user_curr].cexp_set.intersection(*cexp_set);
					if( user_info[user_curr].cexp_set.empty() )
						user_info[user_curr].connect_type = yes_connected;
				}
				else	user_info[user_curr].connect_type = yes_connected;
				if( user_set )
					user_r_set[user_i].insert(0);
				if( user_bool )
					user_r_bool[user_i] = true;
				if( user_pack )
					user_r_pack[user_i] = true;
				break;

				default:
				CPPAD_ASSERT_UNKNOWN(false);
			}
			// drop into op = UsrrpOp code to handle case where user_i == 0
			// for both UsrrvOp and UsrrpOp together.

			case UsrrpOp:
			if( op == UsrrpOp )
			{	// parameter result in an atomic operation sequence
				CPPAD_ASSERT_UNKNOWN( user_state == user_ret );
				CPPAD_ASSERT_UNKNOWN( 0 < user_i && user_i <= user_m );
				CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 );
				CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_par );
				--user_i;
			}
			if( user_i == 0 )
			{	// call users function for this operation
				user_atom->set_id(user_id);
				bool flag = false;
				if( user_set )
				{	flag = user_atom->
						rev_sparse_jac(user_q, user_r_set, user_s_set);
				}
				if( user_bool )
				{	flag = user_atom->
						rev_sparse_jac(user_q, user_r_bool, user_s_bool);
				}
				if( user_pack )
				{	flag = user_atom->
						rev_sparse_jac(user_q, user_r_pack, user_s_pack);
				}
				if( ! flag )
				{	std::string s =
						"Optimizing an ADFun object"
						" that contains the atomic function\n\t";
					s += user_atom->afun_name();
					s += "\nCurrent atomic_sparsity is set to";
					//
					if( user_set )
						s += "set_sparsity_enum.\n";
					if( user_bool )
						s += "bool_sparsity_enum.\n";
					if( user_pack )
						s += "pack_sparsity_enum.\n";
					//
					s += "This version of rev_sparse_jac returned false";
					CPPAD_ASSERT_KNOWN(false, s.c_str() );
				}
				user_state = user_arg;
			}
			break;
			// ============================================================

			// all cases should be handled above
			default:
			CPPAD_ASSERT_UNKNOWN(0);
		}
	}
	// values corresponding to BeginOp
	CPPAD_ASSERT_UNKNOWN( i_op == 0 && i_var == 0 && op == BeginOp );
	tape[i_var].op           = op;
	tape[i_var].connect_type = yes_connected;
	// -------------------------------------------------------------

	// Determine which variables can be conditionally skipped
	for(i = 0; i < num_var; i++)
	{	if( tape[i].connect_type == cexp_connected &&
		  ! cexp_vec_set[i].empty() )
		{	std::set<class_cexp_pair>::const_iterator itr =
				cexp_vec_set[i].begin();
			while( itr != cexp_vec_set[i].end() )
			{	j = itr->index();
				if( itr->compare() == true )
					cskip_info[j].skip_var_false.push_back(i);
				else cskip_info[j].skip_var_true.push_back(i);
				itr++;
			}
		}
	}
	// Determine size of skip information in user_info
	for(i = 0; i < user_info.size(); i++)
	{	if( user_info[i].connect_type == cexp_connected &&
		  ! user_info[i].cexp_set.empty() )
		{	std::set<class_cexp_pair>::const_iterator itr =
				user_info[i].cexp_set.begin();
			while( itr != user_info[i].cexp_set.end() )
			{	j = itr->index();
				if( itr->compare() == true )
					cskip_info[j].n_op_false =
						user_info[i].op_end - user_info[i].op_begin;
				else
					cskip_info[j].n_op_true =
						user_info[i].op_end - user_info[i].op_begin;
				itr++;
			}
		}
	}

	// Sort the conditional skip information by the maximum of the
	// index for the left and right comparision operands
	CppAD::vector<size_t> cskip_info_order( cskip_info.size() );
	{	CppAD::vector<size_t> keys( cskip_info.size() );
		for(i = 0; i < cskip_info.size(); i++)
			keys[i] = std::max( cskip_info[i].left, cskip_info[i].right );
		CppAD::index_sort(keys, cskip_info_order);
	}
	// index in sorted order
	size_t cskip_order_next = 0;
	// index in order during reverse sweep
	size_t cskip_info_index = cskip_info.size();


	// Initilaize table mapping hash code to variable index in tape
	// as pointing to the BeginOp at the beginning of the tape
	CppAD::vector<size_t>  hash_table_var(CPPAD_HASH_TABLE_SIZE);
	for(i = 0; i < CPPAD_HASH_TABLE_SIZE; i++)
		hash_table_var[i] = 0;
	CPPAD_ASSERT_UNKNOWN( tape[0].op == BeginOp );

	// initialize mapping from old variable index to new
	// operator and variable index
	for(i = 0; i < num_var; i++)
	{	tape[i].new_op  = 0;       // invalid index (except for BeginOp)
		tape[i].new_var = num_var; // invalid index
	}

	// Erase all information in the old recording
	rec->free();

	// initialize mapping from old VecAD index to new VecAD index
	CppAD::vector<size_t> new_vecad_ind(num_vecad_ind);
	for(i = 0; i < num_vecad_ind; i++)
		new_vecad_ind[i] = num_vecad_ind; // invalid index

	j = 0;     // index into the old set of indices
	for(i = 0; i < num_vecad_vec; i++)
	{	// length of this VecAD
		size_t length = play->GetVecInd(j);
		if( vecad_connect[i] != not_connected )
		{	// Put this VecAD vector in new recording
			CPPAD_ASSERT_UNKNOWN(length < num_vecad_ind);
			new_vecad_ind[j] = rec->PutVecInd(length);
			for(k = 1; k <= length; k++) new_vecad_ind[j+k] =
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

	// start playing the operations in the forward direction
	play->forward_start(op, arg, i_op, i_var);
	CPPAD_ASSERT_UNKNOWN( user_curr == user_info.size() );

	// playing forward skips BeginOp at the beginning, but not EndOp at
	// the end.  Put BeginOp at beginning of recording
	CPPAD_ASSERT_UNKNOWN( op == BeginOp );
	CPPAD_ASSERT_NARG_NRES(BeginOp, 1, 1);
	tape[i_var].new_op  = rec->num_op_rec();
	tape[i_var].new_var = rec->PutOp(BeginOp);
	rec->PutArg(0);


	// temporary buffer for new argument values
	addr_t new_arg[6];

	// temporary work space used by record_csum
	// (decalared here to avoid realloaction of memory)
	struct_csum_stacks csum_work;

	// tempory used to hold a size_pair
	struct_size_pair size_pair;

	user_state = user_start;
	while(op != EndOp)
	{	// next op
		play->forward_next(op, arg, i_op, i_var);
		CPPAD_ASSERT_UNKNOWN( (i_op > n)  | (op == InvOp) );
		CPPAD_ASSERT_UNKNOWN( (i_op <= n) | (op != InvOp) );

		// determine if we should insert a conditional skip here
		bool skip = cskip_order_next < cskip_info.size();
		skip     &= op != BeginOp;
		skip     &= op != InvOp;
		skip     &= user_state == user_start;
		if( skip )
		{	j     = cskip_info_order[cskip_order_next];
			if( NumRes(op) > 0 )
				skip &= cskip_info[j].max_left_right < i_var;
			else
				skip &= cskip_info[j].max_left_right <= i_var;
		}
		if( skip )
		{	cskip_order_next++;
			struct_cskip_info info = cskip_info[j];
			size_t n_true  = info.skip_var_true.size() + info.n_op_true;
			size_t n_false = info.skip_var_false.size() + info.n_op_false;
			skip &= n_true > 0 || n_false > 0;
			if( skip )
			{	CPPAD_ASSERT_UNKNOWN( NumRes(CSkipOp) == 0 );
				size_t n_arg   = 7 + n_true + n_false;
				// reserve space for the arguments to this operator but
				// delay setting them until we have all the new addresses
				cskip_info[j].i_arg = rec->ReserveArg(n_arg);
				CPPAD_ASSERT_UNKNOWN( cskip_info[j].i_arg > 0 );
				rec->PutOp(CSkipOp);
			}
		}

		// determine if we should keep this operation in the new
		// operation sequence
		bool keep;
		switch( op )
		{	// see wish_list/Optimize/CompareChange entry.
			case EqpvOp:
			case EqvvOp:
			case LepvOp:
			case LevpOp:
			case LevvOp:
			case LtpvOp:
			case LtvpOp:
			case LtvvOp:
			case NepvOp:
			case NevvOp:
			keep = true;
			break;

			case PriOp:
			keep = false;
			break;

			case InvOp:
			case EndOp:
			keep = true;
			break;

			case StppOp:
			case StvpOp:
			case StpvOp:
			case StvvOp:
			CPPAD_ASSERT_UNKNOWN( NumRes(op) == 0 );
			i = vecad[ arg[0] - 1 ];
			keep = vecad_connect[i] != not_connected;
			break;

			case AddpvOp:
			case AddvvOp:
			case SubpvOp:
			case SubvpOp:
			case SubvvOp:
			keep  = tape[i_var].connect_type != not_connected;
			keep &= tape[i_var].connect_type != csum_connected;
			break;

			case UserOp:
			case UsrapOp:
			case UsravOp:
			case UsrrpOp:
			case UsrrvOp:
			keep = true;
			break;

			default:
			keep = tape[i_var].connect_type != not_connected;
			break;
		}

		unsigned short code         = 0;
		bool           replace_hash = false;
		addr_t         match_var;
		tape[i_var].match = false;
		if( keep ) switch( op )
		{
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
			match_var = unary_match(
				tape                ,  // inputs
				i_var               ,
				play->num_par_rec() ,
				play->GetPar()      ,
				hash_table_var      ,
				code                  // outputs
			);
			if( match_var > 0 )
			{	tape[i_var].match     = true;
				tape[match_var].match = true;
				tape[i_var].new_var   = tape[match_var].new_var;
			}
			else
			{
				replace_hash = true;
				new_arg[0]   = tape[ arg[0] ].new_var;
				rec->PutArg( new_arg[0] );
				tape[i_var].new_op  = rec->num_op_rec();
				tape[i_var].new_var = i = rec->PutOp(op);
				CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < i );
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
			if( tape[arg[0]].connect_type == csum_connected )
			{
				// convert to a sequence of summation operators
				size_pair = record_csum(
					tape                , // inputs
					i_var               ,
					play->num_par_rec() ,
					play->GetPar()      ,
					rec                 ,
					csum_work
				);
				tape[i_var].new_op  = size_pair.i_op;
				tape[i_var].new_var = size_pair.i_var;
				// abort rest of this case
				break;
			}
			case DivvpOp:
			case PowvpOp:
			case ZmulvpOp:
			match_var = binary_match(
				tape                ,  // inputs
				i_var               ,
				play->num_par_rec() ,
				play->GetPar()      ,
				hash_table_var      ,
				code                  // outputs
			);
			if( match_var > 0 )
			{	tape[i_var].match     = true;
				tape[match_var].match = true;
				tape[i_var].new_var   = tape[match_var].new_var;
			}
			else
			{	size_pair = record_vp(
					tape                , // inputs
					i_var               ,
					play->num_par_rec() ,
					play->GetPar()      ,
					rec                 ,
					op                  ,
					arg
				);
				tape[i_var].new_op  = size_pair.i_op;
				tape[i_var].new_var = size_pair.i_var;
				replace_hash = true;
			}
			break;
			// ---------------------------------------------------
			// Binary operators where
			// left is an index and right is a variable
			case DisOp:
			match_var = binary_match(
				tape                ,  // inputs
				i_var               ,
				play->num_par_rec() ,
				play->GetPar()      ,
				hash_table_var      ,
				code                  // outputs
			);
			if( match_var > 0 )
			{	tape[i_var].match     = true;
				tape[match_var].match = true;
				tape[i_var].new_var   = tape[match_var].new_var;
			}
			else
			{	new_arg[0] = arg[0];
				new_arg[1] = tape[ arg[1] ].new_var;
				rec->PutArg( new_arg[0], new_arg[1] );
				tape[i_var].new_op  = rec->num_op_rec();
				tape[i_var].new_var = rec->PutOp(op);
				CPPAD_ASSERT_UNKNOWN(
					new_arg[1] < tape[i_var].new_var
				);
				replace_hash = true;
			}
			break;

			// ---------------------------------------------------
			// Binary operators where
			// left is a parameter and right is a variable
			case SubpvOp:
			case AddpvOp:
			if( tape[arg[1]].connect_type == csum_connected )
			{
				// convert to a sequence of summation operators
				size_pair = record_csum(
					tape                , // inputs
					i_var               ,
					play->num_par_rec() ,
					play->GetPar()      ,
					rec                 ,
					csum_work
				);
				tape[i_var].new_op  = size_pair.i_op;
				tape[i_var].new_var = size_pair.i_var;
				// abort rest of this case
				break;
			}
			case DivpvOp:
			case MulpvOp:
			case PowpvOp:
			case ZmulpvOp:
			match_var = binary_match(
				tape                ,  // inputs
				i_var               ,
				play->num_par_rec() ,
				play->GetPar()      ,
				hash_table_var      ,
				code                  // outputs
			);
			if( match_var > 0 )
			{	tape[i_var].match     = true;
				tape[match_var].match = true;
				tape[i_var].new_var   = tape[match_var].new_var;
			}
			else
			{	size_pair = record_pv(
					tape                , // inputs
					i_var               ,
					play->num_par_rec() ,
					play->GetPar()      ,
					rec                 ,
					op                  ,
					arg
				);
				tape[i_var].new_op  = size_pair.i_op;
				tape[i_var].new_var = size_pair.i_var;
				replace_hash = true;
			}
			break;
			// ---------------------------------------------------
			// Binary operator where
			// both operators are variables
			case AddvvOp:
			case SubvvOp:
			if( (tape[arg[0]].connect_type == csum_connected) |
			    (tape[arg[1]].connect_type == csum_connected)
			)
			{
				// convert to a sequence of summation operators
				size_pair = record_csum(
					tape                , // inputs
					i_var               ,
					play->num_par_rec() ,
					play->GetPar()      ,
					rec                 ,
					csum_work
				);
				tape[i_var].new_op  = size_pair.i_op;
				tape[i_var].new_var = size_pair.i_var;
				// abort rest of this case
				break;
			}
			case DivvvOp:
			case MulvvOp:
			case PowvvOp:
			case ZmulvvOp:
			match_var = binary_match(
				tape                ,  // inputs
				i_var               ,
				play->num_par_rec() ,
				play->GetPar()      ,
				hash_table_var      ,
				code                  // outputs
			);
			if( match_var > 0 )
			{	tape[i_var].match     = true;
				tape[match_var].match = true;
				tape[i_var].new_var   = tape[match_var].new_var;
			}
			else
			{	size_pair = record_vv(
					tape                , // inputs
					i_var               ,
					play->num_par_rec() ,
					play->GetPar()      ,
					rec                 ,
					op                  ,
					arg
				);
				tape[i_var].new_op  = size_pair.i_op;
				tape[i_var].new_var = size_pair.i_var;
				replace_hash = true;
			}
			break;
			// ---------------------------------------------------
			// Conditional expression operators
			case CExpOp:
			CPPAD_ASSERT_NARG_NRES(op, 6, 1);
			new_arg[0] = arg[0];
			new_arg[1] = arg[1];
			mask = 1;
			for(i = 2; i < 6; i++)
			{	if( arg[1] & mask )
				{	new_arg[i] = tape[arg[i]].new_var;
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
			tape[i_var].new_op  = rec->num_op_rec();
			tape[i_var].new_var = rec->PutOp(op);
			//
			// The new addresses for left and right are used during
			// fill in the arguments for the CSkip operations. This does not
			// affect max_left_right which is used during this sweep.
			CPPAD_ASSERT_UNKNOWN( cskip_info_index > 0 );
			cskip_info_index--;
			cskip_info[ cskip_info_index ].left  = new_arg[2];
			cskip_info[ cskip_info_index ].right = new_arg[3];
			break;
			// ---------------------------------------------------
			// Operations with no arguments and no results
			case EndOp:
			CPPAD_ASSERT_NARG_NRES(op, 0, 0);
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
			new_arg[1] = tape[arg[1]].new_var;
			rec->PutArg(new_arg[0], new_arg[1]);
			rec->PutOp(op);
			break;
			//
			case LevpOp:
			case LtvpOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 0);
			new_arg[0] = tape[arg[0]].new_var;
			new_arg[1] = rec->PutPar( play->GetPar(arg[1]) );
			rec->PutArg(new_arg[0], new_arg[1]);
			rec->PutOp(op);
			break;
			//
			case LevvOp:
			case LtvvOp:
			case EqvvOp:
			case NevvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 0);
			new_arg[0] = tape[arg[0]].new_var;
			new_arg[1] = tape[arg[1]].new_var;
			rec->PutArg(new_arg[0], new_arg[1]);
			rec->PutOp(op);
			break;

			// ---------------------------------------------------
			// Operations with no arguments and one result
			case InvOp:
			CPPAD_ASSERT_NARG_NRES(op, 0, 1);
			tape[i_var].new_op  = rec->num_op_rec();
			tape[i_var].new_var = rec->PutOp(op);
			break;
			// ---------------------------------------------------
			// Operations with one argument that is a parameter
			case ParOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1);
			new_arg[0] = rec->PutPar( play->GetPar(arg[0] ) );

			rec->PutArg( new_arg[0] );
			tape[i_var].new_op  = rec->num_op_rec();
			tape[i_var].new_var = rec->PutOp(op);
			break;
			// ---------------------------------------------------
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
			tape[i_var].new_op  = rec->num_op_rec();
			tape[i_var].new_var = rec->PutLoadOp(op);
			break;
			// ---------------------------------------------------
			// Load using a variable index
			case LdvOp:
			CPPAD_ASSERT_NARG_NRES(op, 3, 1);
			new_arg[0] = new_vecad_ind[ arg[0] ];
			new_arg[1] = tape[arg[1]].new_var;
			new_arg[2] = rec->num_load_op_rec();
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < num_vecad_ind );
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[1]) < num_var );
			rec->PutArg(
				new_arg[0],
				new_arg[1],
				new_arg[2]
			);
			tape[i_var].new_var = rec->num_op_rec();
			tape[i_var].new_var = rec->PutLoadOp(op);
			break;
			// ---------------------------------------------------
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
			rec->PutOp(op);
			break;
			// ---------------------------------------------------
			// Store a parameter using a variable index
			case StvpOp:
			CPPAD_ASSERT_NARG_NRES(op, 3, 0);
			new_arg[0] = new_vecad_ind[ arg[0] ];
			new_arg[1] = tape[arg[1]].new_var;
			new_arg[2] = rec->PutPar( play->GetPar(arg[2]) );
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < num_vecad_ind );
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[1]) < num_var );
			rec->PutArg(
				new_arg[0],
				new_arg[1],
				new_arg[2]
			);
			rec->PutOp(op);
			break;
			// ---------------------------------------------------
			// Store a variable using a parameter index
			case StpvOp:
			CPPAD_ASSERT_NARG_NRES(op, 3, 0);
			new_arg[0] = new_vecad_ind[ arg[0] ];
			new_arg[1] = rec->PutPar( play->GetPar(arg[1]) );
			new_arg[2] = tape[arg[2]].new_var;
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < num_vecad_ind );
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[1]) < num_var );
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[2]) < num_var );
			rec->PutArg(
				new_arg[0],
				new_arg[1],
				new_arg[2]
			);
			rec->PutOp(op);
			break;
			// ---------------------------------------------------
			// Store a variable using a variable index
			case StvvOp:
			CPPAD_ASSERT_NARG_NRES(op, 3, 0);
			new_arg[0] = new_vecad_ind[ arg[0] ];
			new_arg[1] = tape[arg[1]].new_var;
			new_arg[2] = tape[arg[2]].new_var;
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < num_vecad_ind );
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[1]) < num_var );
			CPPAD_ASSERT_UNKNOWN( size_t(new_arg[2]) < num_var );
			rec->PutArg(
				new_arg[0],
				new_arg[1],
				new_arg[2]
			);
			rec->PutOp(op);
			break;

			// -----------------------------------------------------------
			case UserOp:
			CPPAD_ASSERT_NARG_NRES(op, 4, 0);
			if( user_state == user_start )
			{	user_state = user_arg;
				CPPAD_ASSERT_UNKNOWN( user_curr > 0 );
				user_curr--;
				user_info[user_curr].op_begin = rec->num_op_rec();
			}
			else
			{	user_state = user_start;
				user_info[user_curr].op_end = rec->num_op_rec() + 1;
			}
			// user_index, user_id, user_n, user_m
			if( user_info[user_curr].connect_type != not_connected )
			{	rec->PutArg(arg[0], arg[1], arg[2], arg[3]);
				rec->PutOp(UserOp);
			}
			break;

			case UsrapOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 0);
			if( user_info[user_curr].connect_type != not_connected )
			{	new_arg[0] = rec->PutPar( play->GetPar(arg[0]) );
				rec->PutArg(new_arg[0]);
				rec->PutOp(UsrapOp);
			}
			break;

			case UsravOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 0);
			if( user_info[user_curr].connect_type != not_connected )
			{	new_arg[0] = tape[arg[0]].new_var;
				if( size_t(new_arg[0]) < num_var )
				{	rec->PutArg(new_arg[0]);
					rec->PutOp(UsravOp);
				}
				else
				{	// This argument does not affect the result and
					// has been optimized out so use nan in its place.
					new_arg[0] = rec->PutPar( base_nan );
					rec->PutArg(new_arg[0]);
					rec->PutOp(UsrapOp);
				}
			}
			break;

			case UsrrpOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 0);
			if( user_info[user_curr].connect_type != not_connected )
			{	new_arg[0] = rec->PutPar( play->GetPar(arg[0]) );
				rec->PutArg(new_arg[0]);
				rec->PutOp(UsrrpOp);
			}
			break;

			case UsrrvOp:
			CPPAD_ASSERT_NARG_NRES(op, 0, 1);
			if( user_info[user_curr].connect_type != not_connected )
			{	tape[i_var].new_op  = rec->num_op_rec();
				tape[i_var].new_var = rec->PutOp(UsrrvOp);
			}
			break;
			// ---------------------------------------------------

			// all cases should be handled above
			default:
			CPPAD_ASSERT_UNKNOWN(false);

		}
		if( replace_hash )
		{	// The old variable index i_var corresponds to the
			// new variable index tape[i_var].new_var. In addition
			// this is the most recent variable that has this code.
			hash_table_var[code] = i_var;
		}

	}
	// modify the dependent variable vector to new indices
	for(i = 0; i < dep_taddr.size(); i++ )
	{	CPPAD_ASSERT_UNKNOWN( size_t(tape[dep_taddr[i]].new_var) < num_var );
		dep_taddr[i] = tape[ dep_taddr[i] ].new_var;
	}

# ifndef NDEBUG
	size_t num_new_op = rec->num_op_rec();
	for(i_var = 0; i_var < tape.size(); i_var++)
		CPPAD_ASSERT_UNKNOWN( tape[i_var].new_op < num_new_op );
# endif

	// Move skip information from user_info to cskip_info
	for(i = 0; i < user_info.size(); i++)
	{	if( user_info[i].connect_type == cexp_connected &&
		  ! user_info[i].cexp_set.empty() )
		{	std::set<class_cexp_pair>::const_iterator itr =
				user_info[i].cexp_set.begin();
			while( itr != user_info[i].cexp_set.end() )
			{	j = itr->index();
				k = user_info[i].op_begin;
				while(k < user_info[i].op_end)
				{	if( itr->compare() == true )
						cskip_info[j].skip_op_false.push_back(k++);
					else	cskip_info[j].skip_op_true.push_back(k++);
				}
				itr++;
			}
		}
	}

	// fill in the arguments for the CSkip operations
	CPPAD_ASSERT_UNKNOWN( cskip_order_next == cskip_info.size() );
	for(i = 0; i < cskip_info.size(); i++)
	{	struct_cskip_info info = cskip_info[i];
		if( info.i_arg > 0 )
		{	CPPAD_ASSERT_UNKNOWN( info.n_op_true==info.skip_op_true.size() );
			CPPAD_ASSERT_UNKNOWN(info.n_op_false==info.skip_op_false.size());
			size_t n_true  =
				info.skip_var_true.size() + info.skip_op_true.size();
			size_t n_false =
				info.skip_var_false.size() + info.skip_op_false.size();
			size_t i_arg   = info.i_arg;
			rec->ReplaceArg(i_arg++, info.cop   );
			rec->ReplaceArg(i_arg++, info.flag  );
			rec->ReplaceArg(i_arg++, info.left  );
			rec->ReplaceArg(i_arg++, info.right );
			rec->ReplaceArg(i_arg++, n_true     );
			rec->ReplaceArg(i_arg++, n_false    );
			for(j = 0; j < info.skip_var_true.size(); j++)
			{	i_var = info.skip_var_true[j];
				if( tape[i_var].match )
				{	// The operation for this argument has been removed,
					// so use an operator index that never comes up.
					rec->ReplaceArg(i_arg++, rec->num_op_rec());
				}
				else
				{	CPPAD_ASSERT_UNKNOWN( tape[i_var].new_op > 0 );
					rec->ReplaceArg(i_arg++, tape[i_var].new_op );
				}
			}
			for(j = 0; j < info.skip_op_true.size(); j++)
			{	i_op = info.skip_op_true[j];
				rec->ReplaceArg(i_arg++, i_op);
			}
			for(j = 0; j < info.skip_var_false.size(); j++)
			{	i_var = info.skip_var_false[j];
				if( tape[i_var].match )
				{	// The operation for this argument has been removed,
					// so use an operator index that never comes up.
					rec->ReplaceArg(i_arg++, rec->num_op_rec());
				}
				else
				{	CPPAD_ASSERT_UNKNOWN( tape[i_var].new_op > 0 );
					rec->ReplaceArg(i_arg++, tape[i_var].new_op );
				}
			}
			for(j = 0; j < info.skip_op_false.size(); j++)
			{	i_op = info.skip_op_false[j];
				rec->ReplaceArg(i_arg++, i_op);
			}
			rec->ReplaceArg(i_arg++, n_true + n_false);
# ifndef NDEBUG
			size_t n_arg   = 7 + n_true + n_false;
			CPPAD_ASSERT_UNKNOWN( info.i_arg + n_arg == i_arg );
# endif
		}
	}
}

} // END_CPPAD_OPTIMIZE_NAMESPACE

/*!
Optimize a player object operation sequence

The operation sequence for this object is replaced by one with fewer operations
but the same funcition and derivative values.

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< \a Base > and computations by this routine are done using type
\a Base.

\param options
The default value for this option is the empty string.
The only other possible value is "no_conditional_skip".
If this option is present, no conditional skip operators will be generated.

*/
template <class Base>
void ADFun<Base>::optimize(const std::string& options)
{	// place to store the optimized version of the recording
	recorder<Base> rec;

	// number of independent variables
	size_t n = ind_taddr_.size();

# ifndef NDEBUG
	size_t i, j, m = dep_taddr_.size();
	CppAD::vector<Base> x(n), y(m), check(m);
	Base max_taylor(0);
	bool check_zero_order = num_order_taylor_ > 0;
	if( check_zero_order )
	{	// zero order coefficients for independent vars
		for(j = 0; j < n; j++)
		{	CPPAD_ASSERT_UNKNOWN( play_.GetOp(j+1) == InvOp );
			CPPAD_ASSERT_UNKNOWN( ind_taddr_[j]    == j+1   );
			x[j] = taylor_[ ind_taddr_[j] * cap_order_taylor_ + 0];
		}
		// zero order coefficients for dependent vars
		for(i = 0; i < m; i++)
		{	CPPAD_ASSERT_UNKNOWN( dep_taddr_[i] < num_var_tape_  );
			y[i] = taylor_[ dep_taddr_[i] * cap_order_taylor_ + 0];
		}
		// maximum zero order coefficient not counting BeginOp at beginning
		// (which is correpsonds to uninitialized memory).
		for(i = 1; i < num_var_tape_; i++)
		{	if(  abs_geq(taylor_[i*cap_order_taylor_+0] , max_taylor) )
				max_taylor = taylor_[i*cap_order_taylor_+0];
		}
	}
# endif

	// create the optimized recording
	CppAD::optimize::optimize_run<Base>(options, n, dep_taddr_, &play_, &rec);

	// number of variables in the recording
	num_var_tape_  = rec.num_var_rec();

	// now replace the recording
	play_.get(rec);

	// set flag so this function knows it has been optimized
	has_been_optimized_ = true;

	// free memory allocated for sparse Jacobian calculation
	// (the results are no longer valid)
	for_jac_sparse_pack_.resize(0, 0);
	for_jac_sparse_set_.resize(0,0);

	// free old Taylor coefficient memory
	taylor_.free();
	num_order_taylor_     = 0;
	cap_order_taylor_     = 0;

	// resize and initilaize conditional skip vector
	// (must use player size because it now has the recoreder information)
	cskip_op_.erase();
	cskip_op_.extend( play_.num_op_rec() );

# ifndef NDEBUG
	if( check_zero_order )
	{
		// zero order forward calculation using new operation sequence
		check = Forward(0, x);

		// check results
		Base eps = 10. * CppAD::numeric_limits<Base>::epsilon();
		for(i = 0; i < m; i++) CPPAD_ASSERT_KNOWN(
			abs_geq( eps * max_taylor , check[i] - y[i] ) ,
			"Error during check of f.optimize()."
		);

		// Erase memory that this calculation was done so NDEBUG gives
		// same final state for this object (from users perspective)
		num_order_taylor_     = 0;
	}
# endif
}

} // END_CPPAD_NAMESPACE
# endif
