// $Id: color_general.hpp 3757 2015-11-30 12:03:07Z bradbell $
# ifndef CPPAD_COLOR_GENERAL_HPP
# define CPPAD_COLOR_GENERAL_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    Eclipse Public License Version 1.0.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */
# include <cppad/local/cppad_colpack.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file color_general.hpp
Coloring algorithm for a general sparse matrix.
*/
// --------------------------------------------------------------------------
/*!
Determine which rows of a general sparse matrix can be computed together;
i.e., do not have non-zero entries with the same column index.

\tparam VectorSize
is a simple vector class with elements of type size_t.

\tparam VectorSet
is an unspecified type with the exception that it must support the
operations under pattern and the following operations where
p is a VectorSet object:
\n
<code>VectorSet p</code>
Constructs a new vector of sets object.
\n
<code>p.resize(ns, ne)</code>
resizes \c p to \c ns sets with elements between zero \c ne.
All of the \c ns sets are initially empty.
\n
<code>p.add_element(s, e)</code>
add element \c e to set with index \c s.

\param pattern [in]
Is a representation of the sparsity pattern for the matrix.
Note that color_general does not change the values in pattern,
but it is not const because its iterator facility modifies some of its
internal data.
\n
<code>m = pattern.n_set()</code>
\n
sets \c m to the number of rows in the sparse matrix.
All of the row indices are less than this value.
\n
<code>n = pattern.end()</code>
\n
sets \c n to the number of columns in the sparse matrix.
All of the column indices are less than this value.
\n
<code>pattern.begin(i)</code>
instructs the iterator facility to start iterating over
columns in the i-th row of the sparsity pattern.
\n
<code>j = pattern.next_element()</code>
Sets j to the next possibly non-zero column
in the row specified by the previous call to <code>pattern.begin</code>.
If there are no more such columns, the value
<code>pattern.end()</code> is returned.

\param row [in]
is a vector specifying which row indices to compute.

\param col [in]
is a vector, with the same size as row,
that specifies which column indices to compute.
For each  valid index k, the index pair
<code>(row[k], col[k])</code> must be present in the sparsity pattern.
It may be that some entries in the sparsity pattern do not need to be computed;
i.e, do not appear in the set of
<code>(row[k], col[k])</code> entries.

\param color [out]
is a vector with size m.
The input value of its elements does not matter.
Upon return, it is a coloring for the rows of the sparse matrix.
\n
\n
If for some i, <code>color[i] == m</code>, then
the i-th row does not appear in the vector row.
Otherwise, <code>color[i] < m</code>.
\n
\n
Suppose two differen rows, <code>i != r</code> have the same color and
column index j is such that both of the pairs
<code>(i, j)</code> and <code>(r, j)</code> appear in the sparsity pattern.
It follows that neither of these pairs appear in the set of
<code>(row[k], col[k])</code> entries.
\n
\n
This routine tries to minimize, with respect to the choice of colors,
the maximum, with respct to k, of <code>color[ row[k] ]</code>
(not counting the indices k for which row[k] == m).
*/
template <class VectorSet, class VectorSize>
void color_general_cppad(
	      VectorSet&        pattern ,
	const VectorSize&       row     ,
	const VectorSize&       col     ,
	CppAD::vector<size_t>&  color   )
{	size_t i, j, k, ell, r;

	size_t K = row.size();
	size_t m = pattern.n_set();
	size_t n = pattern.end();

	CPPAD_ASSERT_UNKNOWN( size_t( col.size() )   == K );
	CPPAD_ASSERT_UNKNOWN( size_t( color.size() ) == m );

	// We define the set of rows, columns, and pairs that appear
	// by the set ( row[k], col[k] ) for k = 0, ... , K-1.

	// initialize rows that appear
	CppAD::vector<bool> row_appear(m);
	for(i = 0; i < m; i++)
			row_appear[i] = false;

	// rows and columns that appear
	VectorSet c2r_appear, r2c_appear;
	c2r_appear.resize(n, m);
	r2c_appear.resize(m, n);
	for(k = 0;  k < K; k++)
	{	CPPAD_ASSERT_UNKNOWN( pattern.is_element(row[k], col[k]) );
		row_appear[ row[k] ] = true;
		c2r_appear.add_element(col[k], row[k]);
		r2c_appear.add_element(row[k], col[k]);
	}

	// for each column, which rows are non-zero and do not appear
	VectorSet not_appear;
	not_appear.resize(n, m);
	for(i = 0; i < m; i++)
	{	pattern.begin(i);
		j = pattern.next_element();
		while( j != pattern.end() )
		{	if( ! c2r_appear.is_element(j , i) )
				not_appear.add_element(j, i);
			j = pattern.next_element();
		}
	}

	// initial coloring
	color.resize(m);
	ell = 0;
	for(i = 0; i < m; i++)
	{	if( row_appear[i] )
			color[i] = ell++;
		else	color[i] = m;
	}
	/*
	See GreedyPartialD2Coloring Algorithm Section 3.6.2 of
	Graph Coloring in Optimization Revisited by
	Assefaw Gebremedhin, Fredrik Maane, Alex Pothen

	The algorithm above was modified (by Brad Bell) to take advantage of the
	fact that only the entries (subset of the sparsity pattern) specified by
	row and col need to be computed.
	*/
	CppAD::vector<bool> forbidden(m);
	for(i = 1; i < m; i++) // for each row that appears
	if( color[i] < m )
	{
		// initial all colors as ok for this row
		// (value of forbidden for ell > initial color[i] does not matter)
		for(ell = 0; ell <= color[i]; ell++)
			forbidden[ell] = false;

		// -----------------------------------------------------
		// Forbid colors for which this row would destroy results:
		//
		// for each column that is non-zero for this row
		pattern.begin(i);
		j = pattern.next_element();
		while( j != pattern.end() )
		{	// for each row that appears with this column
			c2r_appear.begin(j);
			r = c2r_appear.next_element();
			while( r != c2r_appear.end() )
			{	// if this is not the same row, forbid its color
				if( (r < i) & (color[r] < m) )
					forbidden[ color[r] ] = true;
				r = c2r_appear.next_element();
			}
			j = pattern.next_element();
		}


		// -----------------------------------------------------
		// Forbid colors that destroy results needed for this row.
		//
		// for each column that appears with this row
		r2c_appear.begin(i);
		j = r2c_appear.next_element();
		while( j != r2c_appear.end() )
		{	// For each row that is non-zero for this column
			// (the appear rows have already been checked above).
			not_appear.begin(j);
			r = not_appear.next_element();
			while( r != not_appear.end() )
			{	// if this is not the same row, forbid its color
				if( (r < i) & (color[r] < m) )
					forbidden[ color[r] ] = true;
				r = not_appear.next_element();
			}
			j = r2c_appear.next_element();
		}

		// pick the color with smallest index
		ell = 0;
		while( forbidden[ell] )
		{	ell++;
			CPPAD_ASSERT_UNKNOWN( ell <= color[i] );
		}
		color[i] = ell;
	}
	return;
}

# if CPPAD_HAS_COLPACK
/*!
Colpack version of determining which rows of a sparse matrix
can be computed together.

\copydetails color_general
*/
template <class VectorSet, class VectorSize>
void color_general_colpack(
	      VectorSet&        pattern ,
	const VectorSize&       row     ,
	const VectorSize&       col     ,
	CppAD::vector<size_t>&  color   )
{	size_t i, j, k;
	size_t m = pattern.n_set();
	size_t n = pattern.end();

	// Determine number of non-zero entries in each row
	CppAD::vector<size_t> n_nonzero(m);
	size_t n_nonzero_total = 0;
	for(i = 0; i < m; i++)
	{	n_nonzero[i] = 0;
		pattern.begin(i);
		j = pattern.next_element();
		while( j != pattern.end() )
		{	n_nonzero[i]++;
			j = pattern.next_element();
		}
		n_nonzero_total += n_nonzero[i];
	}

	// Allocate memory and fill in Adolc sparsity pattern
	CppAD::vector<unsigned int*> adolc_pattern(m);
	CppAD::vector<unsigned int>  adolc_memory(m + n_nonzero_total);
	size_t i_memory = 0;
	for(i = 0; i < m; i++)
	{	adolc_pattern[i]    = adolc_memory.data() + i_memory;
		adolc_pattern[i][0] = n_nonzero[i];
		pattern.begin(i);
		j = pattern.next_element();
		k = 1;
		while(j != pattern.end() )
		{	adolc_pattern[i][k++] = j;
			j = pattern.next_element();
		}
		CPPAD_ASSERT_UNKNOWN( k == 1 + n_nonzero[i] );
		i_memory += k;
	}
	CPPAD_ASSERT_UNKNOWN( i_memory == m + n_nonzero_total );

	// Must use an external routine for this part of the calculation because
	// ColPack/ColPackHeaders.h has as 'using namespace std' at global level.
	cppad_colpack_general(color, m, n, adolc_pattern);

	return;
}
# endif // CPPAD_HAS_COLPACK

} // END_CPPAD_NAMESPACE
# endif
