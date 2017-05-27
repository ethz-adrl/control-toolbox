// $Id: sparse_pattern.hpp 3757 2015-11-30 12:03:07Z bradbell $
# ifndef CPPAD_SPARSE_PATTERN_HPP
# define CPPAD_SPARSE_PATTERN_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    Eclipse Public License Version 1.0.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

// necessary definitions
# include <cppad/local/define.hpp>
# include <cppad/local/sparse_pack.hpp>
# include <cppad/local/sparse_set.hpp>
# include <cppad/local/sparse_list.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file sparse_pattern.hpp
Determine internal spasity pattern from correpsonding element type.
*/

/*!
Template structure used obtain the internal sparsity pattern type
form the corresponding element type.
The general form is not valid, must use a specialization.

\tparam Element_type
type of an element in the sparsity structrue.

\par <code>internal_sparsity<Element_type>::pattern_type</code>
is the type of the corresponding internal sparsity pattern.
*/
template <class Element_type> struct internal_sparsity;
/*!
Specilization for \c bool elements.
*/
template <>
struct internal_sparsity<bool>
{
	typedef sparse_pack pattern_type;
};
/*!
Specilization for <code>std::set<size_t></code> elements.
*/
template <>
struct internal_sparsity< std::set<size_t> >
{
	typedef CPPAD_INTERNAL_SPARSE_SET pattern_type;
};

} // END_CPPAD_NAMESPACE

# endif
