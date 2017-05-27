// $Id: for_sparse_jac.hpp 3757 2015-11-30 12:03:07Z bradbell $
# ifndef CPPAD_FOR_SPARSE_JAC_HPP
# define CPPAD_FOR_SPARSE_JAC_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    Eclipse Public License Version 1.0.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

/*
$begin ForSparseJac$$
$spell
	std
	var
	Jacobian
	Jac
	const
	Bool
	proportional
	VecAD
	CondExpRel
	optimizer
	cpp
$$

$section Jacobian Sparsity Pattern: Forward Mode$$

$head Syntax$$
$icode%s% = %f%.ForSparseJac(%q%, %r%)
%$$
$icode%s% = %f%.ForSparseJac(%q%, %r%, %transpose%, %dependency%)%$$

$head Purpose$$
We use $latex F : B^n \rightarrow B^m$$ to denote the
$cref/AD function/glossary/AD Function/$$ corresponding to $icode f$$.
For a fixed $latex n \times q$$ matrix $latex R$$,
the Jacobian of $latex F[ x + R * u ]$$
with respect to $latex u$$ at $latex u = 0$$ is
$latex \[
	S(x) = F^{(1)} ( x ) * R
\] $$
Given a
$cref/sparsity pattern/glossary/Sparsity Pattern/$$
for $latex R$$,
$code ForSparseJac$$ returns a sparsity pattern for the $latex S(x)$$.

$head f$$
The object $icode f$$ has prototype
$codei%
	ADFun<%Base%> %f%
%$$
Note that the $cref ADFun$$ object $icode f$$ is not $code const$$.
After a call to $code ForSparseJac$$, the sparsity pattern
for each of the variables in the operation sequence
is held in $icode f$$ (for possible later use by $cref RevSparseHes$$).
These sparsity patterns are stored with elements of type $code bool$$
or elements of type $code std::set<size_t>$$
(see $cref/VectorSet/ForSparseJac/VectorSet/$$ below).

$subhead size_forward_bool$$
After $code ForSparseJac$$, if $icode k$$ is a $code size_t$$ object,
$codei%
	%k% = %f%.size_forward_bool()
%$$
sets $icode k$$ to the amount of memory (in unsigned character units)
used to store the sparsity pattern with elements of type $code bool$$
in the function object $icode f$$.
If the sparsity patterns for the previous $code ForSparseJac$$ used
elements of type $code bool$$,
the return value for $code size_forward_bool$$ will be non-zero.
Otherwise, its return value will be zero.
This sparsity pattern is stored for use by $cref RevSparseHes$$ and
when it is not longer needed, it can be deleted
(and the corresponding memory freed) using
$codei%
	%f%.size_forward_bool(0)
%$$
After this call, $icode%f%.size_forward_bool()%$$ will return zero.

$subhead size_forward_set$$
After $code ForSparseJac$$, if $icode k$$ is a $code size_t$$ object,
$codei%
	%k% = %f%.size_forward_set()
%$$
sets $icode s$$ to the total number of elements in all the sets corresponding
to the sparsity pattern stored in the function object $icode f$$.
If the sparsity patterns for this operation use elements of type $code bool$$,
the return value for $code size_forward_set$$ will be zero.
Otherwise, its return value will be non-zero
(unless the entire sparsity pattern is false).
This sparsity pattern is stored for use by $cref RevSparseHes$$ and
when it is not longer needed, it can be deleted
(and the corresponding memory freed) using
$codei%
	%f%.size_forward_set(0)
%$$
After this call, $icode%f%.size_forward_set()%$$ will return zero.

$head x$$
the sparsity pattern is valid for all values of the independent
variables in $latex x \in B^n$$
(even if it has $cref CondExp$$ or $cref VecAD$$ operations).

$head q$$
The argument $icode q$$ has prototype
$codei%
	size_t %q%
%$$
It specifies the number of columns in
$latex R \in B^{n \times q}$$ and the Jacobian
$latex S(x) \in B^{m \times q}$$.

$head transpose$$
The argument $icode transpose$$ has prototype
$codei%
	bool %transpose%
%$$
The default value $code false$$ is used when $icode transpose$$ is not present.

$head dependency$$
The argument $icode dependency$$ has prototype
$codei%
	bool %dependency%
%$$
If $icode dependency$$ is true,
the $cref/dependency pattern/dependency.cpp/Dependency Pattern/$$
(instead of sparsity pattern) is computed.

$head r$$
The argument $icode r$$ has prototype
$codei%
	const %VectorSet%& %r%
%$$
see $cref/VectorSet/ForSparseJac/VectorSet/$$ below.

$subhead transpose false$$
If $icode r$$ has elements of type $code bool$$,
its size is $latex n * q$$.
If it has elements of type $code std::set<size_t>$$,
its size is $latex n$$ and all the set elements must be between
zero and $icode%q%-1%$$ inclusive.
It specifies a
$cref/sparsity pattern/glossary/Sparsity Pattern/$$
for the matrix $latex R \in B^{n \times q}$$.

$subhead transpose true$$
If $icode r$$ has elements of type $code bool$$,
its size is $latex q * n$$.
If it has elements of type $code std::set<size_t>$$,
its size is $latex q$$ and all the set elements must be between
zero and $icode%n%-1%$$ inclusive.
It specifies a
$cref/sparsity pattern/glossary/Sparsity Pattern/$$
for the matrix $latex R^\R{T} \in B^{q \times n}$$.

$head s$$
The return value $icode s$$ has prototype
$codei%
	%VectorSet% %s%
%$$
see $cref/VectorSet/ForSparseJac/VectorSet/$$ below.

$subhead transpose false$$
If $icode s$$ has elements of type $code bool$$,
its size is $latex m * q$$.
If it has elements of type $code std::set<size_t>$$,
its size is $latex m$$ and all its set elements are between
zero and $icode%q%-1%$$ inclusive.
It specifies a
$cref/sparsity pattern/glossary/Sparsity Pattern/$$
for the matrix $latex S(x) \in B^{m \times q}$$.

$subhead transpose true$$
If $icode s$$ has elements of type $code bool$$,
its size is $latex q * m$$.
If it has elements of type $code std::set<size_t>$$,
its size is $latex q$$ and all its set elements are between
zero and $icode%m%-1%$$ inclusive.
It specifies a
$cref/sparsity pattern/glossary/Sparsity Pattern/$$
for the matrix $latex S(x)^\R{T} \in B^{q \times m}$$.

$head VectorSet$$
The type $icode VectorSet$$ must be a $cref SimpleVector$$ class with
$cref/elements of type/SimpleVector/Elements of Specified Type/$$
$code bool$$ or $code std::set<size_t>$$;
see $cref/sparsity pattern/glossary/Sparsity Pattern/$$ for a discussion
of the difference.

$head Entire Sparsity Pattern$$
Suppose that $latex q = n$$ and
$latex R$$ is the $latex n \times n$$ identity matrix.
In this case,
the corresponding value for $icode s$$ is a
sparsity pattern for the Jacobian $latex S(x) = F^{(1)} ( x )$$.

$head Example$$
$children%
	example/for_sparse_jac.cpp
%$$
The file
$cref for_sparse_jac.cpp$$
contains an example and test of this operation.
It returns true if it succeeds and false otherwise.
The file
$cref/sparsity_sub.cpp/sparsity_sub.cpp/ForSparseJac/$$
contains an example and test of using $code ForSparseJac$$
to compute the sparsity pattern for a subset of the Jacobian.

$end
-----------------------------------------------------------------------------
*/

# include <cppad/local/std_set.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file for_sparse_jac.hpp
Forward mode Jacobian sparsity patterns.
*/

// ---------------------------------------------------------------------------
/*!
Calculate Jacobian vector of bools sparsity patterns using forward mode.

The C++ source code corresponding to this operation is
\verbatim
	s = f.ForSparseJac(q, r)
\endverbatim

\tparam Base
is the base type for this recording.

\tparam VectorSet
is a simple vector class with elements of type \c bool.

\param transpose
are the sparsity patterns transposed.

\param dependency
Are the derivatives with respect to left and right of the expression below
considered to be non-zero:
\code
    CondExpRel(left, right, if_true, if_false)
\endcode
This is used by the optimizer to obtain the correct dependency relations.

\param q
is the number of columns in the matrix \f$ R \f$.

\param r
is a sparsity pattern for the matrix \f$ R \f$.

\param s
The input value of \a s must be a vector with size \c m*q
where \c m is the number of dependent variables
corresponding to the operation sequence stored in \a play.
The input value of the components of \c s does not matter.
On output, \a s is the sparsity pattern for the matrix
\f[
	S(x) = F^{(1)} (x) * R
\f]
where \f$ F \f$ is the function corresponding to the operation sequence
and \a x is any argument value.

\param total_num_var
is the total number of variable in this recording.

\param dep_taddr
maps dependendent variable index
to the corresponding variable in the tape.

\param ind_taddr
maps independent variable index
to the corresponding variable in the tape.

\param play
is the recording that defines the function we are computing the sparsity
pattern for.

\param for_jac_sparsity
the input value of \a for_jac_sparsity does not matter.
On output, \a for_jac_sparsity.n_set() == \a total_num_var
and \a for_jac_sparsity.end() == \a q.
It contains the forward sparsity pattern for all of the variables on the
tape (given the sparsity pattern for the independent variables is \f$ R \f$).
*/
template <class Base, class VectorSet>
void ForSparseJacBool(
	bool                   transpose        ,
	bool                   dependency       ,
	size_t                 q                ,
	const VectorSet&       r                ,
	VectorSet&             s                ,
	size_t                 total_num_var    ,
	CppAD::vector<size_t>& dep_taddr        ,
	CppAD::vector<size_t>& ind_taddr        ,
	CppAD::player<Base>&   play             ,
	sparse_pack&           for_jac_sparsity )
{
	// temporary indices
	size_t i, j;

	// range and domain dimensions for F
	size_t m = dep_taddr.size();
	size_t n = ind_taddr.size();

	CPPAD_ASSERT_KNOWN(
		q > 0,
		"ForSparseJac: q is not greater than zero"
	);
	CPPAD_ASSERT_KNOWN(
		size_t(r.size()) == n * q,
		"ForSparseJac: size of r is not equal to\n"
		"q times domain dimension for ADFun object."
	);

	// allocate memory for the requested sparsity calculation result
	for_jac_sparsity.resize(total_num_var, q);

	// set values corresponding to independent variables
	for(i = 0; i < n; i++)
	{	CPPAD_ASSERT_UNKNOWN( ind_taddr[i] < total_num_var );
		// ind_taddr[i] is operator taddr for i-th independent variable
		CPPAD_ASSERT_UNKNOWN( play.GetOp( ind_taddr[i] ) == InvOp );

		// set bits that are true
		if( transpose )
		{	for(j = 0; j < q; j++) if( r[ j * n + i ] )
				for_jac_sparsity.add_element( ind_taddr[i], j);
		}
		else
		{	for(j = 0; j < q; j++) if( r[ i * q + j ] )
				for_jac_sparsity.add_element( ind_taddr[i], j);
		}
	}

	// evaluate the sparsity patterns
	ForJacSweep(
		dependency,
		n,
		total_num_var,
		&play,
		for_jac_sparsity
	);

	// return values corresponding to dependent variables
	CPPAD_ASSERT_UNKNOWN( size_t(s.size()) == m * q );
	for(i = 0; i < m; i++)
	{	CPPAD_ASSERT_UNKNOWN( dep_taddr[i] < total_num_var );

		// extract the result from for_jac_sparsity
		if( transpose )
		{	for(j = 0; j < q; j++)
				s[ j * m + i ] = false;
		}
		else
		{	for(j = 0; j < q; j++)
				s[ i * q + j ] = false;
		}
		CPPAD_ASSERT_UNKNOWN( for_jac_sparsity.end() == q );
		for_jac_sparsity.begin( dep_taddr[i] );
		j = for_jac_sparsity.next_element();
		while( j < q )
		{	if( transpose )
				s[j * m + i] = true;
			else	s[i * q + j] = true;
			j = for_jac_sparsity.next_element();
		}
	}
}

/*!
Calculate Jacobian vector of sets sparsity patterns using forward mode.

The C++ source code corresponding to this operation is
\verbatim
	s = f.ForSparseJac(q, r)
\endverbatim

\tparam Base
see \c SparseJacBool.

\tparam VectorSet
is a simple vector class with elements of type \c std::set<size_t>.

\param transpose
see \c SparseJacBool.

\param dependency
see \c SparseJacBool.

\param q
see \c SparseJacBool.

\param r
see \c SparseJacBool.

\param s
see \c SparseJacBool.

\param total_num_var
see \c SparseJacBool.

\param dep_taddr
see \c SparseJacBool.

\param ind_taddr
see \c SparseJacBool.

\param play
see \c SparseJacBool.

\param for_jac_sparsity
see \c SparseJacBool.
*/

template <class Base, class VectorSet>
void ForSparseJacSet(
	bool                        transpose        ,
	bool                        dependency       ,
	size_t                      q                ,
	const VectorSet&            r                ,
	VectorSet&                  s                ,
	size_t                      total_num_var    ,
	CppAD::vector<size_t>&      dep_taddr        ,
	CppAD::vector<size_t>&      ind_taddr        ,
	CppAD::player<Base>&        play             ,
	CPPAD_INTERNAL_SPARSE_SET&  for_jac_sparsity )
{
	// temporary indices
	size_t i, j;
	std::set<size_t>::const_iterator itr;

	// range and domain dimensions for F
	size_t m = dep_taddr.size();
	size_t n = ind_taddr.size();

	CPPAD_ASSERT_KNOWN(
		q > 0,
		"RevSparseJac: q is not greater than zero"
	);
	CPPAD_ASSERT_KNOWN(
		size_t(r.size()) == n || transpose,
		"RevSparseJac: size of r is not equal to n and transpose is false."
	);
	CPPAD_ASSERT_KNOWN(
		size_t(r.size()) == q || ! transpose,
		"RevSparseJac: size of r is not equal to q and transpose is true."
	);

	// allocate memory for the requested sparsity calculation
	for_jac_sparsity.resize(total_num_var, q);

	// set values corresponding to independent variables
	if( transpose )
	{	for(i = 0; i < q; i++)
		{	// add the elements that are present
			itr = r[i].begin();
			while( itr != r[i].end() )
			{	j = *itr++;
				CPPAD_ASSERT_KNOWN(
				j < n,
				"ForSparseJac: transpose is true and element of the set\n"
				"r[j] has value greater than or equal n."
				);
				CPPAD_ASSERT_UNKNOWN( ind_taddr[j] < total_num_var );
				// operator for j-th independent variable
				CPPAD_ASSERT_UNKNOWN( play.GetOp( ind_taddr[j] ) == InvOp );
				for_jac_sparsity.add_element( ind_taddr[j], i);
			}
		}
	}
	else
	{	for(i = 0; i < n; i++)
		{	CPPAD_ASSERT_UNKNOWN( ind_taddr[i] < total_num_var );
			// ind_taddr[i] is operator taddr for i-th independent variable
			CPPAD_ASSERT_UNKNOWN( play.GetOp( ind_taddr[i] ) == InvOp );

			// add the elements that are present
			itr = r[i].begin();
			while( itr != r[i].end() )
			{	j = *itr++;
				CPPAD_ASSERT_KNOWN(
					j < q,
					"ForSparseJac: an element of the set r[i] "
					"has value greater than or equal q."
				);
				for_jac_sparsity.add_element( ind_taddr[i], j);
			}
		}
	}
	// evaluate the sparsity patterns
	ForJacSweep(
		dependency,
		n,
		total_num_var,
		&play,
		for_jac_sparsity
	);

	// return values corresponding to dependent variables
	CPPAD_ASSERT_UNKNOWN( size_t(s.size()) == m || transpose );
	CPPAD_ASSERT_UNKNOWN( size_t(s.size()) == q || ! transpose );
	for(i = 0; i < m; i++)
	{	CPPAD_ASSERT_UNKNOWN( dep_taddr[i] < total_num_var );

		// extract results from for_jac_sparsity
		// and add corresponding elements to sets in s
		CPPAD_ASSERT_UNKNOWN( for_jac_sparsity.end() == q );
		for_jac_sparsity.begin( dep_taddr[i] );
		j = for_jac_sparsity.next_element();
		while( j < q )
		{	if( transpose )
				s[j].insert(i);
			else	s[i].insert(j);
			j = for_jac_sparsity.next_element();
		}
	}
}
// ---------------------------------------------------------------------------
/*!
Private helper function for ForSparseJac(q, r).

All of the description in the public member function ForSparseJac(q, r)
applies.

\param set_type
is a \c bool value. This argument is used to dispatch to the proper source
code depending on the value of \c VectorSet::value_type.

\param transpose
See \c ForSparseJac(q, r, transpose, dependency).

\param dependency
See \c ForSparseJac(q, r, transpose, dependency).

\param q
See \c ForSparseJac(q, r, transpose, dependency).

\param r
See \c ForSparseJac(q, r, transpose, dependency).

\param s
is the return value for the corresponding call to \c ForSparseJac(q, r).
*/

template <class Base>
template <class VectorSet>
void ADFun<Base>::ForSparseJacCase(
	bool                set_type      ,
	bool                transpose     ,
	bool                dependency    ,
	size_t              q             ,
	const VectorSet&    r             ,
	VectorSet&          s             )
{	size_t m = Range();

	// check VectorSet is Simple Vector class with bool elements
	CheckSimpleVector<bool, VectorSet>();

	// dimension size of result vector
	s.resize( m * q );

	// store results in s and for_jac_sparse_pack_
	ForSparseJacBool(
		transpose        ,
		dependency       ,
		q                ,
		r                ,
		s                ,
		num_var_tape_    ,
		dep_taddr_       ,
		ind_taddr_       ,
		play_            ,
		for_jac_sparse_pack_
	);
}


/*!
Private helper function for \c ForSparseJac(q, r).

All of the description in the public member function \c ForSparseJac(q, r)
applies.

\param set_type
is a \c std::set<size_t> object.
This argument is used to dispatch to the proper source
code depending on the value of \c VectorSet::value_type.

\param transpose
See \c ForSparseJac(q, r, transpose, dependency).

\param dependency
See \c ForSparseJac(q, r, transpose, dependency).

\param q
See \c ForSparseJac(q, r, transpose, dependency).

\param r
See \c ForSparseJac(q, r, transpose, dependency).

\param s
is the return value for the corresponding call to \c ForSparseJac(q, r).
*/
template <class Base>
template <class VectorSet>
void ADFun<Base>::ForSparseJacCase(
	const std::set<size_t>&    set_type      ,
	bool                       transpose     ,
	bool                       dependency    ,
	size_t                     q             ,
	const VectorSet&           r             ,
	VectorSet&                 s             )
{	size_t m = Range();

	// check VectorSet is Simple Vector class with sets for elements
	CheckSimpleVector<std::set<size_t>, VectorSet>(
		one_element_std_set<size_t>(), two_element_std_set<size_t>()
	);

	// dimension size of result vector
	if( transpose )
		s.resize(q);
	else	s.resize( m );

	// store results in r and for_jac_sparse_pack_
	CppAD::ForSparseJacSet(
		transpose        ,
		dependency       ,
		q                ,
		r                ,
		s                ,
		num_var_tape_    ,
		dep_taddr_       ,
		ind_taddr_       ,
		play_            ,
		for_jac_sparse_set_
	);
}
// ---------------------------------------------------------------------------

/*!
User API for Jacobian sparsity patterns using forward mode.

The C++ source code corresponding to this operation is
\verbatim
	s = f.ForSparseJac(q, r, transpose, dependency)
\endverbatim

\tparam Base
is the base type for this recording.

\tparam VectorSet
is a simple vector with elements of type \c bool
or \c std::set<size_t>.

\param q
is the number of columns in the matrix \f$ R \f$.

\param r
is a sparsity pattern for the matrix \f$ R \f$.

\param transpose
are sparsity patterns for \f$ R \f$ and \f$ S(x) \f$ transposed.

\param dependency
Are the derivatives with respect to left and right of the expression below
considered to be non-zero:
\code
	CondExpRel(left, right, if_true, if_false)
\endcode
This is used by the optimizer to obtain the correct dependency relations.

\return
The value of \c transpose is false (true),
the return value is a sparsity pattern for \f$ S(x) \f$ (\f$ S(x)^T \f$) where
\f[
	S(x) = F^{(1)} (x) * R
\f]
where \f$ F \f$ is the function corresponding to the operation sequence
and \a x is any argument value.
If \c VectorSet::value_type is \c bool,
the return value has size \f$ m * q \f$ (\f$ q * m \f$).
where \c m is the number of dependent variables
corresponding to the operation sequence stored in \c f.
If \c VectorSet::value_type is \c std::set<size_t>,
the return value has size \f$ m \f$ ( \f$ q \f$ )
and with all its elements between zero and
\f$ q - 1 \f$ ( \f$ m - 1 \f$).

\par Side Effects
If \c VectorSet::value_type is \c bool,
the forward sparsity pattern for all of the variables on the
tape is stored in \c for_jac_sparse_pack__.
In this case
\verbatim
	for_jac_sparse_pack_.n_set() == num_var_tape_
	for_jac_sparse_pack_.end() == q
	for_jac_sparse_set_.n_set()  == 0
	for_jac_sparse_set_.end()  == 0
\endverbatim
\n
\n
If \c VectorSet::value_type is \c std::set<size_t>,
the forward sparsity pattern for all of the variables on the
tape is stored in \c for_jac_sparse_set__.
In this case
\verbatim
	for_jac_sparse_set_.n_set()   == num_var_tape_
	for_jac_sparse_set_.end()   == q
	for_jac_sparse_pack_.n_set()  == 0
	for_jac_sparse_pack_.end()  == 0
\endverbatim
*/
template <class Base>
template <class VectorSet>
VectorSet ADFun<Base>::ForSparseJac(
	size_t             q             ,
	const VectorSet&   r             ,
	bool               transpose     ,
	bool               dependency    )
{	VectorSet s;
	typedef typename VectorSet::value_type Set_type;

	// free all memory currently in sparsity patterns
	for_jac_sparse_pack_.resize(0, 0);
	for_jac_sparse_set_.resize(0, 0);

	ForSparseJacCase(
		Set_type()  ,
		transpose   ,
		dependency  ,
		q           ,
		r           ,
		s
	);

	return s;
}
// ===========================================================================
// ForSparseJacCheckpoint
/*!
Forward mode Jacobian sparsity calculation used by checkpoint functions.

\tparam Base
is the base type for this recording.

\param transpose
is true (false) s is equal to \f$ S(x) \f$ (\f$ S(x)^T \f$)
where
\f[
	S(x) = F^{(1)} (x) * R
\f]
where \f$ F \f$ is the function corresponding to the operation sequence
and \f$ x \f$ is any argument value.

\param q
is the number of columns in the matrix \f$ R \f$.

\param r
is a sparsity pattern for the matrix \f$ R \f$.

\param transpose
are the sparsity patterns for \f$ R \f$ and \f$ S(x) \f$ transposed.

\param dependency
Are the derivatives with respect to left and right of the expression below
considered to be non-zero:
\code
	CondExpRel(left, right, if_true, if_false)
\endcode
This is used by the optimizer to obtain the correct dependency relations.

\param s
The input size and elements of s do not matter.
On output, s is the sparsity pattern for the matrix \f$ S(x) \f$
or \f$ S(x)^T \f$ depending on transpose.

\par Side Effects
If \c VectorSet::value_type is \c bool,
the forward sparsity pattern for all of the variables on the
tape is stored in \c for_jac_sparse_pack__.
In this case
\verbatim
	for_jac_sparse_pack_.n_set() == num_var_tape_
	for_jac_sparse_pack_.end() == q
	for_jac_sparse_set_.n_set()  == 0
	for_jac_sparse_set_.end()  == 0
\endverbatim
\n
\n
If \c VectorSet::value_type is \c std::set<size_t>,
the forward sparsity pattern for all of the variables on the
tape is stored in \c for_jac_sparse_set__.
In this case
\verbatim
	for_jac_sparse_set_.n_set()   == num_var_tape_
	for_jac_sparse_set_.end()   == q
	for_jac_sparse_pack_.n_set()  == 0
	for_jac_sparse_pack_.end()  == 0
\endverbatim
*/
template <class Base>
void ADFun<Base>::ForSparseJacCheckpoint(
	size_t                        q          ,
	CPPAD_INTERNAL_SPARSE_SET&    r          ,
	bool                          transpose  ,
	bool                          dependency ,
	CPPAD_INTERNAL_SPARSE_SET&    s          )
{	size_t n = Domain();
	size_t m = Range();

# ifndef NDEBUG
	if( transpose )
	{	CPPAD_ASSERT_UNKNOWN( r.n_set() == q );
		CPPAD_ASSERT_UNKNOWN( r.end()   == n );
	}
	else
	{	CPPAD_ASSERT_UNKNOWN( r.n_set() == n );
		CPPAD_ASSERT_UNKNOWN( r.end()   == q );
	}
	for(size_t j = 0; j < n; j++)
	{	CPPAD_ASSERT_UNKNOWN( ind_taddr_[j] == (j+1) );
		CPPAD_ASSERT_UNKNOWN( play_.GetOp( ind_taddr_[j] ) == InvOp );
	}
# endif

	// free all memory currently in sparsity patterns
	for_jac_sparse_pack_.resize(0, 0);
	for_jac_sparse_set_.resize(0, 0);

	// allocate new sparsity pattern
	for_jac_sparse_set_.resize(num_var_tape_, q);

	// set sparsity pattern for dependent variables
	if( transpose )
	{	for(size_t i = 0; i < q; i++)
		{	r.begin(i);
			size_t j = r.next_element();
			while( j < n )
			{	for_jac_sparse_set_.add_element( ind_taddr_[j], i );
				j = r.next_element();
			}
		}
	}
	else
	{	for(size_t j = 0; j < n; j++)
		{	r.begin(j);
			size_t i = r.next_element();
			while( i < q )
			{	for_jac_sparse_set_.add_element( ind_taddr_[j], i );
				i = r.next_element();
			}
		}
	}

	// evaluate the sparsity pattern for all variables
	ForJacSweep(
		dependency,
		n,
		num_var_tape_,
		&play_,
		for_jac_sparse_set_
	);

	// dimension the return value
	if( transpose )
		s.resize(q, m);
	else
		s.resize(m, q);

	// return values corresponding to dependent variables
	for(size_t i = 0; i < m; i++)
	{	CPPAD_ASSERT_UNKNOWN( dep_taddr_[i] < num_var_tape_ );

		// extract the result from for_jac_sparse_set_
		CPPAD_ASSERT_UNKNOWN( for_jac_sparse_set_.end() == q );
		for_jac_sparse_set_.begin( dep_taddr_[i] );
		size_t j = for_jac_sparse_set_.next_element();
		while( j < q )
		{	if( transpose )
				s.add_element(j, i);
			else
				s.add_element(i, j);
			j  = for_jac_sparse_set_.next_element();
		}
	}

}


} // END_CPPAD_NAMESPACE
# endif
