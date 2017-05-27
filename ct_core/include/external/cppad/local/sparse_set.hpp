// $Id: sparse_set.hpp 3757 2015-11-30 12:03:07Z bradbell $
# ifndef CPPAD_SPARSE_SET_HPP
# define CPPAD_SPARSE_SET_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    Eclipse Public License Version 1.0.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */
# include <set>
# include <algorithm>
# include <iterator>
# include <cppad/local/cppad_assert.hpp>


namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file sparse_set.hpp
Vector of sets of positive integers stored as std::set<size_t>.
*/

/*!
Vector of sets of positive integers, each set stored as a standard set.
*/

class sparse_set {
private:
	/// type used for each set in the vector sets
	typedef std::set<size_t> Set;
	/// Number of sets that we are representing
	/// (set by constructor and resize).
	size_t n_set_;
	/// Possible elements in each set are 0, 1, ..., end_ - 1
	/// (set by constructor and resize).
	size_t end_;
	/// The vector of sets
	CppAD::vector<Set> data_;
	/// index for which we were retrieving next_element
	/// (use n_set_ if no such index exists).
	size_t next_index_;
	/// Next element that we will return using next_element
	/// (use end_ for no such element exists; i.e., past end of the set).
	Set::iterator next_element_;
public:
	// -----------------------------------------------------------------
	/*! Default constructor (no sets)
	*/
	sparse_set(void) :
	n_set_(0)                     ,
	end_(0)                       ,
	next_index_(0)
	{ }
	// -----------------------------------------------------------------
	/*! Make use of copy constructor an error

	\param v
	vector that we are attempting to make a copy of.
	*/
	sparse_set(const sparse_set& v)
	{	// Error:
		// Probably a sparse_set argument has been passed by value
		CPPAD_ASSERT_UNKNOWN(false);
	}
	// -----------------------------------------------------------------
	/*! Destructor
	*/
	~sparse_set(void)
	{ }
	// -----------------------------------------------------------------
	/*! Change number of sets, set end, and initialize all sets as empty


	If \c n_set_in is zero, any memory currently allocated for this object
	is freed. Otherwise, new memory may be allocated for the sets (if needed).

	\param n_set_in
	is the number of sets in this vector of sets.

	\param end_in
	is the maximum element plus one (the minimum element is 0).
	*/
	void resize(size_t n_set_in, size_t end_in)
	{	n_set_          = n_set_in;
		end_            = end_in;
		if( n_set_ == 0 )
		{	// free all memory connected with data_
			data_.clear();
			return;
		}
		// now start a new vector with empty sets
		data_.resize(n_set_);

		// value that signfies past end of list
		next_index_ = n_set_;
	}
	// -----------------------------------------------------------------
	/*! Add one element to a set.

	\param index
	is the index for this set in the vector of sets.

	\param element
	is the element we are adding to the set.

	\par Checked Assertions
	\li index    < n_set_
	\li element  < end_
	*/
	void add_element(size_t index, size_t element)
	{	// This routine should use the std::insert operator
		// that cashes the iterator of previous insertion for when
		// insertions occur in order. We should speed test that this
		// actually makes things faster.
		CPPAD_ASSERT_UNKNOWN( index   < n_set_ );
		CPPAD_ASSERT_UNKNOWN( element < end_ );
		data_[ index ].insert( element );
	}
	// -----------------------------------------------------------------
	/*! Is an element of a set.

	\param index
	is the index for this set in the vector of sets.

	\param element
	is the element we are adding to the set.

	\par Checked Assertions
	\li index    < n_set_
	\li element  < end_
	*/
	bool is_element(size_t index, size_t element)
	{	// This routine should use the std::insert operator
		// that cashes the iterator of previous insertion for when
		// insertions occur in order. We should speed test that this
		// actually makes things faster.
		CPPAD_ASSERT_UNKNOWN( index   < n_set_ );
		CPPAD_ASSERT_UNKNOWN( element < end_ );
		std::set<size_t>::iterator itr = data_[ index ].find( element );
		return itr != data_[index].end();
	}
	// -----------------------------------------------------------------
	/*! Begin retrieving elements from one of the sets.

	\param index
	is the index for the set that is going to be retrieved.
	The elements of the set are retrieved in increasing order.

	\par Checked Assertions
	\li index  < n_set_
	*/
	void begin(size_t index)
	{	// initialize element to search for in this set
		CPPAD_ASSERT_UNKNOWN( index < n_set_ );
		next_index_       = index;
		next_element_     = data_[index].begin();

		return;
	}
	// -----------------------------------------------------------------
	/*! Get the next element from the current retrieval set.

	\return
	is the next element in the set with index
	specified by the previous call to \c begin.
	If no such element exists, \c this->end() is returned.
	*/
	size_t next_element(void)
	{
		if( next_element_ == data_[next_index_].end() )
			return end_;

		return *next_element_++;
	}
	// -----------------------------------------------------------------
	/*! Assign the empty set to one of the sets.

	\param target
	is the index of the set we are setting to the empty set.

	\par Checked Assertions
	\li target < n_set_
	*/
	void clear(size_t target)
	{	CPPAD_ASSERT_UNKNOWN( target < n_set_ );
		data_[target].clear();
	}
	// -----------------------------------------------------------------
	/*! Assign one set equal to another set.

	\param this_target
	is the index (in this \c sparse_set object) of the set being assinged.

	\param other_value
	is the index (in the other \c sparse_set object) of the
	that we are using as the value to assign to the target set.

	\param other
	is the other \c sparse_set object (which may be the same as this
	\c sparse_set object).

	\par Checked Assertions
	\li this_target  < n_set_
	\li other_value  < other.n_set_
	*/
	void assignment(
		size_t               this_target  ,
		size_t               other_value  ,
		const sparse_set&   other        )
	{	CPPAD_ASSERT_UNKNOWN( this_target  <   n_set_        );
		CPPAD_ASSERT_UNKNOWN( other_value  <   other.n_set_  );

		data_[this_target] = other.data_[other_value];
	}

	// -----------------------------------------------------------------
	/*! Assign a set equal to the union of two other sets.

	\param this_target
	is the index (in this \c sparse_set object) of the set being assinged.

	\param this_left
	is the index (in this \c sparse_set object) of the
	left operand for the union operation.
	It is OK for \a this_target and \a this_left to be the same value.

	\param other_right
	is the index (in the other \c sparse_set object) of the
	right operand for the union operation.
	It is OK for \a this_target and \a other_right to be the same value.

	\param other
	is the other \c sparse_set object (which may be the same as this
	\c sparse_set object).

	\par Checked Assertions
	\li this_target <  n_set_
	\li this_left   <  n_set_
	\li other_right <  other.n_set_
	*/
	void binary_union(
		size_t                  this_target  ,
		size_t                  this_left    ,
		size_t                  other_right  ,
		const sparse_set&      other        )
	{	CPPAD_ASSERT_UNKNOWN( this_target < n_set_         );
		CPPAD_ASSERT_UNKNOWN( this_left   < n_set_         );
		CPPAD_ASSERT_UNKNOWN( other_right < other.n_set_   );

		// use a temporary set for holding results
		// (in case target set is same as one of the other sets)
		Set temp;
		std::set_union(
			data_[this_left].begin()         ,
			data_[this_left].end()           ,
			other.data_[other_right].begin() ,
			other.data_[other_right].end()   ,
			std::inserter(temp, temp.begin())
		);

		// move results to the target set with out copying elements
		data_[this_target].swap(temp);

	}
	// -----------------------------------------------------------------
	/*! Sum over all sets of the number of elements

	/return
	The the total number of elements
	*/
	size_t number_elements(void) const
	{	size_t i, count;
		count = 0;
		for(i = 0; i < n_set_; i++)
			count += data_[i].size();
		return count;
	}
	// -----------------------------------------------------------------
	/*! Fetch n_set for vector of sets object.

	\return
	Number of from sets for this vector of sets object
	*/
	size_t n_set(void) const
	{	return n_set_; }
	// -----------------------------------------------------------------
	/*! Fetch end for this vector of sets object.

	\return
	is the maximum element value plus one (the minimum element value is 0).
	*/
	size_t end(void) const
	{	return end_; }
};

/*!
Copy a user vector of sets sparsity pattern to an internal sparse_set object.

\tparam VectorSet
is a simple vector with elements of type \c std::set<size_t>.

\param internal
The input value of sparisty does not matter.
Upon return it contains the same sparsity pattern as \c user
(or the transposed sparsity pattern).

\param user
sparsity pattern that we are placing \c internal.

\param n_row
number of rows in the sparsity pattern in \c user
(range dimension).

\param n_col
number of columns in the sparsity pattern in \c user
(domain dimension).

\param transpose
if true, the sparsity pattern in \c internal is the transpose
of the one in \c user.
Otherwise it is the same sparsity pattern.
*/
template<class VectorSet>
void sparsity_user2internal(
	sparse_set&             internal  ,
	const VectorSet&        user      ,
	size_t                  n_row     ,
	size_t                  n_col     ,
	bool                    transpose )
{
	CPPAD_ASSERT_KNOWN(
		size_t( user.size() ) == n_row,
		"Size of this vector of sets sparsity pattern is not equal\n"
		"the range dimension for the corresponding function."
	);

	size_t i, j;
	std::set<size_t>::const_iterator itr;

	// transposed pattern case
	if( transpose )
	{	internal.resize(n_col, n_row);
		for(i = 0; i < n_row; i++)
		{	itr = user[i].begin();
			while(itr != user[i].end())
			{	j = *itr++;
				CPPAD_ASSERT_KNOWN(
					j < n_col,
					"An element in this vector of sets sparsity pattern "
					"is greater than or equal\n"
					"the domain dimension for the corresponding function."
				);
				internal.add_element(j, i);
			}
		}
		return;
	}

	// same pattern case
	internal.resize(n_row, n_col);
	for(i = 0; i < n_row; i++)
	{	itr = user[i].begin();
		while(itr != user[i].end())
		{	j = *itr++;
			CPPAD_ASSERT_UNKNOWN( j < n_col );
			internal.add_element(i, j);
		}
	}
	return;
}

} // END_CPPAD_NAMESPACE
# endif
