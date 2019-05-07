// $Id: rev_sparse_hes.hpp 3757 2015-11-30 12:03:07Z bradbell $
# ifndef CPPAD_REV_SPARSE_HES_HPP
# define CPPAD_REV_SPARSE_HES_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    Eclipse Public License Version 1.0.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

/*
$begin RevSparseHes$$
$spell
	std
	VecAD
	Jacobian
	Jac
	Hessian
	Hes
	const
	Bool
	Dep
	proportional
	var
	cpp
$$

$section Hessian Sparsity Pattern: Reverse Mode$$

$head Syntax$$
$icode%h% = %f%.RevSparseHes(%q%, %s%)
%$$
$icode%h% = %f%.RevSparseHes(%q%, %s%, %transpose%)%$$


$head Purpose$$
We use $latex F : \B{R}^n \rightarrow \B{R}^m$$ to denote the
$cref/AD function/glossary/AD Function/$$ corresponding to $icode f$$.
For a fixed matrix $latex R \in \B{R}^{n \times q}$$
and a fixed vector $latex S \in \B{R}^{1 \times m}$$,
we define
$latex \[
\begin{array}{rcl}
H(x)
& = & \partial_x \left[ \partial_u S * F[ x + R * u ] \right]_{u=0}
\\
& = & R^\R{T} * (S * F)^{(2)} ( x )
\\
H(x)^\R{T}
& = & (S * F)^{(2)} ( x ) * R
\end{array}
\] $$
Given a
$cref/sparsity pattern/glossary/Sparsity Pattern/$$
for the matrix $latex R$$ and the vector $latex S$$,
$code RevSparseHes$$ returns a sparsity pattern for the $latex H(x)$$.

$head f$$
The object $icode f$$ has prototype
$codei%
	const ADFun<%Base%> %f%
%$$

$head x$$
the sparsity pattern is valid for all values of the independent
variables in $latex x \in \B{R}^n$$
(even if it has $cref CondExp$$ or $cref VecAD$$ operations).

$head q$$
The argument $icode q$$ has prototype
$codei%
	size_t %q%
%$$
It specifies the number of columns in $latex R \in \B{R}^{n \times q}$$
and the number of rows in $latex H(x) \in \B{R}^{q \times n}$$.
It must be the same value as in the previous $cref ForSparseJac$$ call
$codei%
	%f%.ForSparseJac(%q%, %r%, %r_transpose%)
%$$
Note that if $icode r_transpose$$ is true, $icode r$$ in the call above
corresponding to $latex R^\R{T} \in \B{R}^{q \times n}$$

$head transpose$$
The argument $icode transpose$$ has prototype
$codei%
	bool %transpose%
%$$
The default value $code false$$ is used when $icode transpose$$ is not present.


$head r$$
The matrix $latex R$$ is specified by the previous call
$codei%
	%f%.ForSparseJac(%q%, %r%, %transpose%)
%$$
see $cref/r/ForSparseJac/r/$$.
The type of the elements of
$cref/VectorSet/RevSparseHes/VectorSet/$$ must be the
same as the type of the elements of $icode r$$.

$head s$$
The argument $icode s$$ has prototype
$codei%
	const %VectorSet%& %s%
%$$
(see $cref/VectorSet/RevSparseHes/VectorSet/$$ below)
If it has elements of type $code bool$$,
its size is $latex m$$.
If it has elements of type $code std::set<size_t>$$,
its size is one and all the elements of $icode%s%[0]%$$
are between zero and $latex m - 1$$.
It specifies a
$cref/sparsity pattern/glossary/Sparsity Pattern/$$
for the vector $icode S$$.

$head h$$
The result $icode h$$ has prototype
$codei%
	%VectorSet%& %h%
%$$
(see $cref/VectorSet/RevSparseHes/VectorSet/$$ below).

$subhead transpose false$$
If $icode h$$ has elements of type $code bool$$,
its size is $latex q * n$$.
If it has elements of type $code std::set<size_t>$$,
its size is $latex q$$ and all the set elements are between
zero and $icode%n%-1%$$ inclusive.
It specifies a
$cref/sparsity pattern/glossary/Sparsity Pattern/$$
for the matrix $latex H(x)$$.

$subhead transpose true$$
If $icode h$$ has elements of type $code bool$$,
its size is $latex n * q$$.
If it has elements of type $code std::set<size_t>$$,
its size is $latex n$$ and all the set elements are between
zero and $icode%q%-1%$$ inclusive.
It specifies a
$cref/sparsity pattern/glossary/Sparsity Pattern/$$
for the matrix $latex H(x)^\R{T}$$.

$head VectorSet$$
The type $icode VectorSet$$ must be a $cref SimpleVector$$ class with
$cref/elements of type/SimpleVector/Elements of Specified Type/$$
$code bool$$ or $code std::set<size_t>$$;
see $cref/sparsity pattern/glossary/Sparsity Pattern/$$ for a discussion
of the difference.
The type of the elements of
$cref/VectorSet/RevSparseHes/VectorSet/$$ must be the
same as the type of the elements of $icode r$$.

$head Entire Sparsity Pattern$$
Suppose that $latex q = n$$ and
$latex R \in \B{R}^{n \times n}$$ is the $latex n \times n$$ identity matrix.
Further suppose that the $latex S$$ is the $th k$$
$cref/elementary vector/glossary/Elementary Vector/$$; i.e.
$latex \[
S_j = \left\{ \begin{array}{ll}
	1  & {\rm if} \; j = k
	\\
	0  & {\rm otherwise}
\end{array} \right.
\] $$
In this case,
the corresponding value $icode h$$ is a
sparsity pattern for the Hessian matrix
$latex F_k^{(2)} (x) \in \B{R}^{n \times n}$$.

$head Example$$
$children%
	example/rev_sparse_hes.cpp
	%example/sparsity_sub.cpp
%$$
The file
$cref rev_sparse_hes.cpp$$
contains an example and test of this operation.
It returns true if it succeeds and false otherwise.
The file
$cref/sparsity_sub.cpp/sparsity_sub.cpp/RevSparseHes/$$
contains an example and test of using $code RevSparseHes$$
to compute the sparsity pattern for a subset of the Hessian.

$end
-----------------------------------------------------------------------------
*/
# include <algorithm>
# include <cppad/local/pod_vector.hpp>
# include <cppad/local/std_set.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file rev_sparse_hes.hpp
Reverse mode Hessian sparsity patterns.
*/

// ===========================================================================
// RevSparseHesBool
/*!
Calculate Hessian sparsity patterns using reverse mode.

The C++ source code corresponding to this operation is
\verbatim
	h = f.RevSparseHes(q, s)
\endverbatim

\tparam Base
is the base type for this recording.

\tparam VectorSet
is a simple vector with elements of type \c bool.

\param transpose
is true (false) if \c is is equal to \f$ H(x) \f$ (\f$ H(x)^T \f$)
where
\f[
	H(x) = R^T (S * F)^{(2)} (x)
\f]
where \f$ F \f$ is the function corresponding to the operation sequence
and \a x is any argument value.

\param q
is the value of \a q in the
by the previous call of the form
\verbatim
	f.ForSparseJac(q, r)
\endverbatim
The value \c r in this call is a sparsity pattern for the matrix \f$ R \f$.

\param s
is a vector with size \c m that specifies the sparsity pattern
for the vector \f$ S \f$,
where \c m is the number of dependent variables
corresponding to the operation sequence stored in \a play.

\param h
the input value of \a h must be a vector with size \c q*n.
The input value of its elements does not matter.
On output, \a h is the sparsity pattern for the matrix \f$ H(x) \f$
or \f$ H(x)^T \f$ depending on \c transpose.

\param num_var
is the total number of variables in this recording.

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
is a vector of sets containing the
the forward Jacobian sparsity pattern corresponding to
$latex R$$ for all of the variables on the tape.
*/

template <class Base, class VectorSet>
void RevSparseHesBool(
	bool                      transpose         ,
	size_t                    q                 ,
	const VectorSet&          s                 ,
	VectorSet&                h                 ,
	size_t                    num_var           ,
	CppAD::vector<size_t>&    dep_taddr         ,
	CppAD::vector<size_t>&    ind_taddr         ,
	CppAD::player<Base>&      play              ,
	sparse_pack&              for_jac_sparsity  )
{
	// temporary indices
	size_t i, j;

	// check Vector is Simple VectorSet class with bool elements
	CheckSimpleVector<bool, VectorSet>();

	// range and domain dimensions for F
	size_t m = dep_taddr.size();
	size_t n = ind_taddr.size();

	CPPAD_ASSERT_KNOWN(
		q == for_jac_sparsity.end(),
		"RevSparseHes: q is not equal to its value\n"
		"in the previous call to ForSparseJac with this ADFun object."
	);
	CPPAD_ASSERT_KNOWN(
		size_t(s.size()) == m,
		"RevSparseHes: size of s is not equal to\n"
		"range dimension for ADFun object."
	);

	// Array that will hold reverse Jacobian dependency flag.
	// Initialize as true for the dependent variables.
	pod_vector<bool> RevJac;
	RevJac.extend(num_var);
	for(i = 0; i < num_var; i++)
		RevJac[i] = false;
	for(i = 0; i < m; i++)
	{	CPPAD_ASSERT_UNKNOWN( dep_taddr[i] < num_var );
		RevJac[ dep_taddr[i] ] = s[i];
	}

	// vector of sets that will hold reverse Hessain values
	sparse_pack rev_hes_sparsity;
	rev_hes_sparsity.resize(num_var, q);

	// compute the Hessian sparsity patterns
	RevHesSweep(
		n,
		num_var,
		&play,
		for_jac_sparsity,
		RevJac.data(),
		rev_hes_sparsity
	);

	// return values corresponding to independent variables
	CPPAD_ASSERT_UNKNOWN( size_t(h.size()) == n * q );
	for(j = 0; j < n; j++)
	{	for(i = 0; i < q; i++)
		{	if( transpose )
				h[ j * q + i ] = false;
			else	h[ i * n + j ] = false;
		}
	}

	// j is index corresponding to reverse mode partial
	for(j = 0; j < n; j++)
	{	CPPAD_ASSERT_UNKNOWN( ind_taddr[j] < num_var );

		// ind_taddr[j] is operator taddr for j-th independent variable
		CPPAD_ASSERT_UNKNOWN( ind_taddr[j] == j + 1 );
		CPPAD_ASSERT_UNKNOWN( play.GetOp( ind_taddr[j] ) == InvOp );

		// extract the result from rev_hes_sparsity
		CPPAD_ASSERT_UNKNOWN( rev_hes_sparsity.end() == q );
		rev_hes_sparsity.begin(j + 1);
		i = rev_hes_sparsity.next_element();
		while( i < q )
		{	if( transpose )
				h[ j * q + i ] = true;
			else	h[ i * n + j ] = true;
			i = rev_hes_sparsity.next_element();
		}
	}

	return;
}

// ===========================================================================
// RevSparseHesSet
/*!
Calculate Hessian sparsity patterns using reverse mode.

The C++ source code corresponding to this operation is
\verbatim
	h = f.RevSparseHes(q, s)
\endverbatim

\tparam Base
is the base type for this recording.

\tparam VectorSet
is a simple vector with elements of type \c std::set<size_t>.

\param transpose
is true (false) if \c is is equal to \f$ H(x) \f$ (\f$ H(x)^T \f$)
where
\f[
	H(x) = R^T (S * F)^{(2)} (x)
\f]
where \f$ F \f$ is the function corresponding to the operation sequence
and \a x is any argument value.

\param q
is the value of \a q in the
by the previous call of the form
\verbatim
	f.ForSparseJac(q, r)
\endverbatim
The value \c r in this call is a sparsity pattern for the matrix \f$ R \f$.

\param s
is a vector with size \c m that specifies the sparsity pattern
for the vector \f$ S \f$,
where \c m is the number of dependent variables
corresponding to the operation sequence stored in \a play.

\param h
If \c transpose, the input value of \a h must be a vector with size \a q.
Otherwise, its input value must have size \c n;
On input, each element of \a h must be an empty set.
On output, \a h is the sparsity pattern for the matrix \f$ H(x) \f$
or \f$ H(x)^T \f$ depending on \c transpose.

\param num_var
is the total number of variables in this recording.

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
is a vector of sets containing the
the forward Jacobian sparsity pattern corresponding to
$latex R$$ for all of the variables on the tape.
*/

template <class Base, class VectorSet>
void RevSparseHesSet(
	bool                       transpose         ,
	size_t                     q                 ,
	const  VectorSet&          s                 ,
	VectorSet&                 h                 ,
	size_t                     num_var           ,
	CppAD::vector<size_t>&     dep_taddr         ,
	CppAD::vector<size_t>&     ind_taddr         ,
	CppAD::player<Base>&       play              ,
	CPPAD_INTERNAL_SPARSE_SET& for_jac_sparsity  )
{
	// temporary indices
	size_t i, j;
	std::set<size_t>::const_iterator itr;

	// check VectorSet is Simple Vector class with sets for elements
	CheckSimpleVector<std::set<size_t>, VectorSet>(
		one_element_std_set<size_t>(), two_element_std_set<size_t>()
	);

	// range and domain dimensions for F
# ifndef NDEBUG
	size_t m = dep_taddr.size();
# endif
	size_t n = ind_taddr.size();

	CPPAD_ASSERT_KNOWN(
		q == for_jac_sparsity.end(),
		"RevSparseHes: q is not equal to its value\n"
		"in the previous call to ForSparseJac with this ADFun object."
	);
	CPPAD_ASSERT_KNOWN(
		s.size() == 1,
		"RevSparseHes: size of s is not equal to one."
	);

	// Array that will hold reverse Jacobian dependency flag.
	// Initialize as true for the dependent variables.
	pod_vector<bool> RevJac;
	RevJac.extend(num_var);
	for(i = 0; i < num_var; i++)
		RevJac[i] = false;
	itr = s[0].begin();
	while( itr != s[0].end() )
	{	i = *itr++;
		CPPAD_ASSERT_KNOWN(
			i < m,
			"RevSparseHes: an element of the set s[0] has value "
			"greater than or equal m"
		);
		CPPAD_ASSERT_UNKNOWN( dep_taddr[i] < num_var );
		RevJac[ dep_taddr[i] ] = true;
	}


	// vector of sets that will hold reverse Hessain values
	CPPAD_INTERNAL_SPARSE_SET rev_hes_sparsity;
	rev_hes_sparsity.resize(num_var, q);

	// compute the Hessian sparsity patterns
	RevHesSweep(
		n,
		num_var,
		&play,
		for_jac_sparsity,
		RevJac.data(),
		rev_hes_sparsity
	);

	// return values corresponding to independent variables
	// j is index corresponding to reverse mode partial
	CPPAD_ASSERT_UNKNOWN( size_t(h.size()) == q || transpose );
	CPPAD_ASSERT_UNKNOWN( size_t(h.size()) == n || ! transpose );
	for(j = 0; j < n; j++)
	{	CPPAD_ASSERT_UNKNOWN( ind_taddr[j] < num_var );
		CPPAD_ASSERT_UNKNOWN( ind_taddr[j] == j + 1 );
		CPPAD_ASSERT_UNKNOWN( play.GetOp( ind_taddr[j] ) == InvOp );

		// extract the result from rev_hes_sparsity
		// and add corresponding elements to result sets in h
		CPPAD_ASSERT_UNKNOWN( rev_hes_sparsity.end() == q );
		rev_hes_sparsity.begin(j+1);
		i = rev_hes_sparsity.next_element();
		while( i < q )
		{	if( transpose )
				h[j].insert(i);
			else	h[i].insert(j);
			i = rev_hes_sparsity.next_element();
		}
	}

	return;
}

// ===========================================================================
// RevSparseHes

/*!
User API for Hessian sparsity patterns using reverse mode.

The C++ source code corresponding to this operation is
\verbatim
	h = f.RevSparseHes(q, r)
\endverbatim

\tparam Base
is the base type for this recording.

\tparam VectorSet
is a simple vector with elements of type \c bool
or \c std::set<size_t>.

\param transpose
is true (false) if \c is is equal to \f$ H(x) \f$ (\f$ H(x)^T \f$)
where
\f[
	H(x) = R^T (S * F)^{(2)} (x)
\f]
where \f$ F \f$ is the function corresponding to the operation sequence
and \a x is any argument value.

\param q
is the value of \a q in the
by the previous call of the form
\verbatim
	f.ForSparseJac(q, r, packed)
\endverbatim
The value \c r in this call is a sparsity pattern for the matrix \f$ R \f$.
The type of the element of \c r for the previous call to \c ForSparseJac
must be the same as the type of the elements of \c s.

\param s
is a vector with size \c m that specifies the sparsity pattern
for the vector \f$ S \f$,
where \c m is the number of dependent variables
corresponding to the operation sequence stored in \a play.

\return
If \c transpose is false (true),
the return vector is a sparsity pattern for \f$ H(x) \f$ (\f$ H(x)^T \f$).
\f[
	H(x) = R^T ( S * F)^{(2)} (x)
\f]
where \f$ F \f$ is the function corresponding to the operation sequence
and \a x is any argument value.
*/

template <class Base>
template <class VectorSet>
VectorSet ADFun<Base>::RevSparseHes(
	size_t q,  const VectorSet& s, bool transpose
)
{	VectorSet h;
	typedef typename VectorSet::value_type Set_type;

	// Should check to make sure q is same as in previous call to
	// forward sparse Jacobian.
	RevSparseHesCase(
		Set_type()    ,
		transpose     ,
		q             ,
		s             ,
		h
	);

	return h;
}
// ===========================================================================
// RevSparseHesCase
/*!
Private helper function for RevSparseHes(q, s).

All of the description in the public member function RevSparseHes(q, s)
applies.

\param set_type
is a \c bool value. This argument is used to dispatch to the proper source
code depending on the vlaue of \c VectorSet::value_type.

\param transpose
See \c RevSparseHes(q, s).

\param q
See \c RevSparseHes(q, s).

\param s
See \c RevSparseHes(q, s).

\param h
is the return value for the corresponging call to \c RevSparseJac(q, s).
*/
template <class Base>
template <class VectorSet>
void ADFun<Base>::RevSparseHesCase(
	bool              set_type         ,
	bool              transpose        ,
	size_t            q                ,
	const VectorSet&  s                ,
	VectorSet&        h                )
{	size_t n = Domain();
	h.resize(q * n );

	CPPAD_ASSERT_KNOWN(
		for_jac_sparse_pack_.n_set() > 0,
		"RevSparseHes: previous stored call to ForSparseJac did not "
		"use bool for the elements of r."
	);
	CPPAD_ASSERT_UNKNOWN( for_jac_sparse_set_.n_set() == 0 );
	CPPAD_ASSERT_UNKNOWN( for_jac_sparse_pack_.n_set() == num_var_tape_  );

	// use sparse_pack for the calculation
	CppAD::RevSparseHesBool(
		transpose                ,
		q                        ,
		s                        ,
		h                        ,
		num_var_tape_            ,
		dep_taddr_               ,
		ind_taddr_               ,
		play_                    ,
		for_jac_sparse_pack_
	);
}
/*!
Private helper function for RevSparseHes(q, s).

All of the description in the public member function RevSparseHes(q, s)
applies.

\param set_type
is a \c std::set<size_t> value.
This argument is used to dispatch to the proper source
code depending on the vlaue of \c VectorSet::value_type.

\param transpose
See \c RevSparseHes(q, s).

\param q
See \c RevSparseHes(q, s).

\param s
See \c RevSparseHes(q, s).

\param h
is the return value for the corresponging call to \c RevSparseJac(q, s).
*/
template <class Base>
template <class VectorSet>
void ADFun<Base>::RevSparseHesCase(
	const std::set<size_t>&   set_type         ,
	bool                      transpose        ,
	size_t                    q                ,
	const VectorSet&          s                ,
	VectorSet&                h                )
{	size_t n = Domain();
	if( transpose )
		h.resize(n);
	else	h.resize(q);

	CPPAD_ASSERT_KNOWN(
		for_jac_sparse_set_.n_set() > 0,
		"RevSparseHes: previous stored call to ForSparseJac did not "
		"use std::set<size_t> for the elements of r."
	);
	CPPAD_ASSERT_UNKNOWN( for_jac_sparse_pack_.n_set() == 0 );
	CPPAD_ASSERT_UNKNOWN( for_jac_sparse_set_.n_set() == num_var_tape_  );

	// use sparse_pack for the calculation
	CppAD::RevSparseHesSet(
		transpose                ,
		q                        ,
		s                        ,
		h                        ,
		num_var_tape_            ,
		dep_taddr_               ,
		ind_taddr_               ,
		play_                    ,
		for_jac_sparse_set_
	);
}
// ===========================================================================
// RevSparseHesCheckpoint
/*!
Hessian sparsity patterns calculation used by checkpoint functions.

\tparam Base
is the base type for this recording.

\param transpose
is true (false) h is equal to \f$ H(x) \f$ (\f$ H(x)^T \f$)
where
\f[
	H(x) = R^T (S * F)^{(2)} (x)
\f]
where \f$ F \f$ is the function corresponding to the operation sequence
and \f$ x \f$ is any argument value.

\param q
is the value of q in the by the previous call of the form
\verbatim
	f.ForSparseJac(q, r)
\endverbatim
The value r in this call is a sparsity pattern for the matrix \f$ R \f$.

\param s
is a vector with size m that specifies the sparsity pattern
for the vector \f$ S \f$,
where m is the number of dependent variables
corresponding to the operation sequence stored in play_.

\param h
The input size and elements of h do not matter.
On output, h is the sparsity pattern for the matrix \f$ H(x) \f$
or \f$ H(x)^T \f$ depending on transpose.

\par Assumptions
The forward jacobian sparsity pattern must be currently stored
in this ADFUN object.
*/
template <class Base>
void ADFun<Base>::RevSparseHesCheckpoint(
	size_t                        q         ,
	vector<bool>&                 s         ,
	bool                          transpose ,
	CPPAD_INTERNAL_SPARSE_SET&    h         )
{	size_t n = Domain();
	size_t m = Range();

	// checkpoint functions should get this right
	CPPAD_ASSERT_UNKNOWN( for_jac_sparse_pack_.n_set() == 0 );
	CPPAD_ASSERT_UNKNOWN( for_jac_sparse_set_.n_set() == num_var_tape_ );
	CPPAD_ASSERT_UNKNOWN( for_jac_sparse_set_.end()   == q );
	CPPAD_ASSERT_UNKNOWN( s.size()                    == m );

	// Array that holds the reverse Jacobiain dependcy flags.
	// Initialize as true for dependent variables, flase for others.
	pod_vector<bool> RevJac;
	RevJac.extend(num_var_tape_);
	for(size_t i = 0; i < num_var_tape_; i++)
		RevJac[i] = false;
	for(size_t i = 0; i < m; i++)
	{	CPPAD_ASSERT_UNKNOWN( dep_taddr_[i] < num_var_tape_ )
		RevJac[ dep_taddr_[i] ] = s[i];
	}

	// holds reverse Hessian sparsity pattern for all variables
	CPPAD_INTERNAL_SPARSE_SET rev_hes_sparsity;
	rev_hes_sparsity.resize(num_var_tape_, q);

	// compute Hessian sparsity pattern for all variables
	RevHesSweep(
		n,
		num_var_tape_,
		&play_,
		for_jac_sparse_set_,
		RevJac.data(),
		rev_hes_sparsity
	);

	// dimension the return value
	if( transpose )
		h.resize(n, q);
	else
		h.resize(q, n);

	// j is index corresponding to reverse mode partial
	for(size_t j = 0; j < n; j++)
	{	CPPAD_ASSERT_UNKNOWN( ind_taddr_[j] < num_var_tape_ );

		// ind_taddr_[j] is operator taddr for j-th independent variable
		CPPAD_ASSERT_UNKNOWN( ind_taddr_[j] == j + 1 );
		CPPAD_ASSERT_UNKNOWN( play_.GetOp( ind_taddr_[j] ) == InvOp );

		// extract the result from rev_hes_sparsity
		CPPAD_ASSERT_UNKNOWN( rev_hes_sparsity.end() == q );
		rev_hes_sparsity.begin(j + 1);
		size_t i = rev_hes_sparsity.next_element();
		while( i < q )
		{	if( transpose )
				h.add_element(j,  i);
			else	h.add_element(i, j);
			i = rev_hes_sparsity.next_element();
		}
	}
}

} // END_CPPAD_NAMESPACE
# endif
