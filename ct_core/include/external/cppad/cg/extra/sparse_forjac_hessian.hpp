#ifndef CPPAD_EXTRA_SPARSE_FORJAC_HESSIAN_INCLUDED
#define CPPAD_EXTRA_SPARSE_FORJAC_HESSIAN_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
 *
 *  CppADCodeGen is distributed under multiple licenses:
 *
 *   - Eclipse Public License Version 1.0 (EPL1), and
 *   - GNU General Public License Version 3 (GPL3).
 *
 *  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
 *  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
 * ----------------------------------------------------------------------------
 * Author: Joao Leal
 */
/**
 * Adapted from CppAD
 */
namespace CppAD {
namespace cg {

/**
 * class used by SparseForJacHessian to hold information relative to the
 * Jacobian so it does not need to be recomputed.
 */
class SparseForjacHessianWorkJac {
public:
    /// version of user row array with the extra value m at end
    std::vector<size_t> user_row;
    /// version of user col array with the extra value n at end
    std::vector<size_t> user_col;
    /// indices that sort the user arrays by row 
    /// with the extra value K at the end
    std::vector<size_t> sort_row;
    /// indices that sort the user arrays by column 
    /// with the extra value K at the end
    std::vector<size_t> sort_col;
    /// number elements in the user sparse Jacobian
    size_t K;

    template<class Base, class VectorSize>
    inline void prepare(const ADFun<Base>& fun,
                        const VectorSize& row,
                        const VectorSize& col) {
        /**
         * Code adapted from ADFun::SparseJacobianForward()
         */
        K = row.size();
        size_t n = fun.Domain();
        size_t m = fun.Range();

        if (user_row.size() == 0) {
            // create version of (row, col, k) sorted by column value
            user_col.resize(K + 1);
            user_row.resize(K + 1);
            sort_col.resize(K + 1);

            // put sorted indices in user_row and user_col
            for (size_t k = 0; k < K; k++) {
                user_row[k] = row[k];
                user_col[k] = col[k];
            }
            user_row[K] = m;
            user_col[K] = n;

            // put sorting indices in sort_col
            index_sort(user_col, sort_col);
        }

#ifndef NDEBUG
        CPPAD_ASSERT_KNOWN(size_t(row.size()) == K && size_t(col.size()) == K,
                           "sparseForJacHessian: either r or c does not have "
                           "the same size as jac.");
        CPPAD_ASSERT_KNOWN(user_row.size() == K + 1 &&
                           user_col.size() == K + 1 &&
                           sort_col.size() == K + 1,
                           "sparseForJacHessian: invalid value in work.");
        for (size_t k = 0; k < K; k++) {
            CPPAD_ASSERT_KNOWN(row[k] < m,
                               "sparseForJacHessian: invalid value in r.");
            CPPAD_ASSERT_KNOWN(col[k] < n,
                               "sparseForJacHessian: invalid value in c.");
            CPPAD_ASSERT_KNOWN(sort_col[k] < K,
                               "sparseForJacHessian: invalid value in work.");
            CPPAD_ASSERT_KNOWN(user_row[k] == row[k],
                               "sparseForJacHessian: invalid value in work.");
            CPPAD_ASSERT_KNOWN(user_col[k] == col[k],
                               "sparseForJacHessian: invalid value in work.");
        }
#endif
    }
    /// inform CppAD that this information needs to be recomputed

    inline void clear(void) {
        user_row.clear();
        user_col.clear();
        sort_row.clear();
        sort_col.clear();
    }
};

/**
 * class used by SparseForJacHessian to hold information relative to the
 * Hessian so it does not need to be recomputed.
 */
class SparseForjacHessianWorkHes {
public:
    /// version of user r array sorted by row or column
    std::vector<size_t> r_sort;
    /// version of user c array sorted by row or column
    std::vector<size_t> c_sort;
    /// mapping from sorted array indices to user array indices
    std::vector<size_t> k_sort;
    /// number elements in the user sparse Hessian
    size_t K;

    template<class Base, class VectorSize>
    inline void prepare(const ADFun<Base>& fun,
                        const VectorSize& row,
                        const VectorSize& col) {
        /**
         * Code adapted from ADFun::SparseHessian()
         */
        size_t n = fun.Domain();
        K = row.size();

        if (r_sort.size() == 0) {
            // create version of (row, col, k) sorted by row value
            c_sort.resize(K);
            r_sort.resize(K + 1);
            k_sort.resize(K);

            // put sorting indices in k_sort
            index_sort(row, k_sort);

            for (size_t k = 0; k < K; k++) {
                r_sort[k] = row[ k_sort[k] ];
                c_sort[k] = col[ k_sort[k] ];
            }
            r_sort[K] = n;
        }
#ifndef NDEBUG
        CPPAD_ASSERT_KNOWN(size_t(row.size()) == K && size_t(col.size()) == K,
                           "sparseForJacHessian: either r or c does not have the same size as ehs.");
        CPPAD_ASSERT_KNOWN(r_sort.size() == K + 1 &&
                           c_sort.size() == K &&
                           k_sort.size() == K,
                           "sparseForJacHessian: invalid value in work.");
        for (size_t k = 0; k < K; k++) {
            CPPAD_ASSERT_KNOWN(row[k] < n,
                               "sparseForJacHessian: invalid value in r.");
            CPPAD_ASSERT_KNOWN(col[k] < n,
                               "sparseForJacHessian: invalid value in c.");
            CPPAD_ASSERT_KNOWN(k_sort[k] < K,
                               "sparseForJacHessian: invalid value in work.");
            CPPAD_ASSERT_KNOWN(r_sort[k] == row[ k_sort[k] ],
                               "sparseForJacHessian: invalid value in work.");
            CPPAD_ASSERT_KNOWN(c_sort[k] == col[ k_sort[k] ],
                               "sparseForJacHessian: invalid value in work.");
        }
#endif
    }
    /// inform CppAD that this information needs to be recomputed

    inline void clear(void) {
        r_sort.clear();
        c_sort.clear();
        k_sort.clear();
    }
};

/**
 * class used by SparseForJacHessian to hold information so it does not need
 * to be recomputed.
 */
class SparseForjacHessianWork {
public:
    SparseForjacHessianWorkJac jac;
    SparseForjacHessianWorkHes hes;
    /// results of the coloring algorithm
    std::vector<size_t> color;

    template<class Base, class VectorSize>
    inline void prepare(const ADFun<Base>& fun,
                        const VectorSize& jacRow,
                        const VectorSize& jacCol,
                        const VectorSize& hesRow,
                        const VectorSize& hesCol) {
        size_t n = fun.Domain();

        CPPAD_ASSERT_KNOWN(color.size() == 0 || color.size() == n,
                           "sparseForJacHessian: invalid value in work.");
        if (color.size() != 0) {
            for (size_t j = 0; j < n; j++) {
                CPPAD_ASSERT_KNOWN(color[j] < n,
                                   "sparseForJacHessian: invalid value in work.");
            }
        }

        jac.prepare(fun, jacRow, jacCol);
        hes.prepare(fun, hesRow, hesCol);
    }
    /// inform CppAD that this information needs to be recomputed

    inline void clear(void) {
        jac.clear();
        hes.clear();
        color.clear();
    }
};

template<class VectorSet>
inline void computeNotUsed(VectorSet& not_used,
                           VectorSet& sparsity,
                           VectorSet& r_used,
                           size_t m,
                           size_t n) {

    assert(not_used.n_set() == 0);
    not_used.resize(m, n);
    
    for (size_t i = 0; i < n; i++) {
        sparsity.begin(i);
        size_t j = sparsity.next_element();
        while (j != sparsity.end()) {
            if (!r_used.is_element(j, i))
                not_used.add_element(j, i);
            j = sparsity.next_element();
        }
    }
}

template<class Base, class VectorSet>
inline size_t colorForwardJacobianHessian(const ADFun<Base>& fun,
                                          const VectorSet& jac_p,
                                          const VectorSet& hes_p,
                                          SparseForjacHessianWork& work) {
    /**
     * Code adapted from ADFun::SparseJacobianForward()
     */

    size_t i, j1, j11, j2, c, k;

    size_t n = fun.Domain();
    size_t m = fun.Range();

    std::vector<size_t>& color = work.color;

    if (color.size() == 0) {

        color.resize(n);

        CPPAD_ASSERT_KNOWN(jac_p.size() == m,
                           "sparseForJacHessian: invalid jacobian sparsity pattern dimension.");
        CPPAD_ASSERT_KNOWN(hes_p.size() == n,
                           "sparseForJacHessian: invalid hessian sparsity pattern dimension.");

        /**
         * Jacobian
         */
        // transpose sparsity pattern
        typedef typename VectorSet::value_type Set_type;
        typedef typename internal_sparsity<Set_type>::pattern_type Pattern_type;
        Pattern_type p_transpose;
        bool transpose = true;
        sparsity_user2internal(p_transpose, jac_p, m, n, transpose);
        //sparsity_user2internal(p_transpose, jac_p, n, m, transpose, "Invalid sparsity pattern");


        size_t jac_K = work.jac.K;
        std::vector<size_t>& jac_row = work.jac.user_row;
        std::vector<size_t>& jac_col = work.jac.user_col;
        std::vector<size_t>& sort_col = work.jac.sort_col;

        CPPAD_ASSERT_UNKNOWN(p_transpose.n_set() == n);
        CPPAD_ASSERT_UNKNOWN(p_transpose.end() == m);

        // rows and columns that are in the returned jacobian
        Pattern_type jac_r_used, jac_c_used;
        jac_r_used.resize(n, m);
        jac_c_used.resize(m, n);

        for (k = 0; k < jac_K; k++) {
            CPPAD_ASSERT_UNKNOWN(jac_row[sort_col[k]] < m && jac_col[sort_col[k]] < n);
            CPPAD_ASSERT_UNKNOWN(k == 0 || jac_col[sort_col[k - 1]] <= jac_col[sort_col[k]]);
            CPPAD_ASSERT_KNOWN(p_transpose.is_element(jac_col[sort_col[k]], jac_row[sort_col[k]]),
                               "sparseForJacHessian: "
                               "a (row, col) pair is not in sparsity pattern.");
            jac_r_used.add_element(jac_col[sort_col[k]], jac_row[sort_col[k]]);
            jac_c_used.add_element(jac_row[sort_col[k]], jac_col[sort_col[k]]);
        }

        // given a row index, which columns are non-zero and not used
        Pattern_type jac_not_used;
        computeNotUsed(jac_not_used, p_transpose, jac_c_used, m, n);

        /**
         * Hessian
         */
        Pattern_type hes_sparsity;
        transpose = false;
        sparsity_user2internal(hes_sparsity, hes_p, n, n, transpose);
        //sparsity_user2internal(hes_sparsity, hes_p, n, n, transpose, "Invalid sparsity pattern");

        size_t hes_K = work.hes.K;
        std::vector<size_t>& hes_row(work.hes.r_sort);
        std::vector<size_t>& hes_col(work.hes.c_sort);

        CPPAD_ASSERT_UNKNOWN(hes_sparsity.n_set() == n);
        CPPAD_ASSERT_UNKNOWN(hes_sparsity.end() == n);

        // rows and columns that are in the returned hessian
        Pattern_type hes_r_used, hes_c_used;
        hes_r_used.resize(n, n);
        hes_c_used.resize(n, n);

        for (k = 0; k < hes_K; k++) {
            CPPAD_ASSERT_UNKNOWN(hes_row[k] < n && hes_col[k] < n);
            CPPAD_ASSERT_UNKNOWN(k == 0 || hes_row[k - 1] <= hes_row[k]);
            CPPAD_ASSERT_KNOWN(hes_sparsity.is_element(hes_row[k], hes_col[k]),
                               "sparseForJacHessian: a (row, col) pair is not in sparsity pattern.");
            hes_r_used.add_element(hes_col[k], hes_row[k]);
            hes_c_used.add_element(hes_row[k], hes_col[k]);
        }

        // given a column index, which rows are non-zero and not used
        Pattern_type hes_not_used;
        computeNotUsed(hes_not_used, hes_sparsity, hes_r_used, n, n);

        // initial coloring
        for (j1 = 0; j1 < n; j1++) {
            color[j1] = j1;
        }

        // See GreedyPartialD2Coloring Algorithm Section 3.6.2 of
        // Graph Coloring in Optimization Revisited by
        // Assefaw Gebremedhin, Fredrik Maane, Alex Pothen
        vectorBool forbidden(n);
        for (j1 = 1; j1 < n; j1++) {
            // initialize all colors as ok for this column
            // (value of forbidden for c > j does not matter)
            for (c = 0; c <= j1; c++)
                forbidden[c] = false;

            /**
             * Jacobian
             */
            // for each row that is non-zero for this column
            p_transpose.begin(j1);
            i = p_transpose.next_element();
            while (i != p_transpose.end()) {
                // for each column that this row uses
                jac_c_used.begin(i);
                j11 = jac_c_used.next_element();
                while (j11 != jac_c_used.end()) {
                    // if this is not the same column, forbid its color
                    if (j11 < j1)
                        forbidden[ color[j11] ] = true;
                    j11 = jac_c_used.next_element();
                }
                i = p_transpose.next_element();
            }

            // for each row that this column uses
            jac_r_used.begin(j1);
            i = jac_r_used.next_element();
            while (i != jac_r_used.end()) {
                // For each column that is non-zero for this row
                // (the used columns have already been checked above).
                jac_not_used.begin(i);
                j11 = jac_not_used.next_element();
                while (j11 != jac_not_used.end()) {
                    // if this is not the same column, forbid its color
                    if (j11 < j1)
                        forbidden[ color[j11] ] = true;
                    j11 = jac_not_used.next_element();
                }
                i = jac_r_used.next_element();
            }

            /**
             * Hessian
             */
            // -----------------------------------------------------
            // Forbid colors that this row would destroy results for.
            // for each column that is non-zero for this row
            hes_sparsity.begin(j1);
            j2 = hes_sparsity.next_element();
            while (j2 != hes_sparsity.end()) {
                // for each row that this column uses
                hes_r_used.begin(j2);
                j11 = hes_r_used.next_element();
                while (j11 != hes_r_used.end()) {
                    // if this is not the same row, forbid its color
                    if (j11 < j1)
                        forbidden[ color[j11] ] = true;
                    j11 = hes_r_used.next_element();
                }
                j2 = hes_sparsity.next_element();
            }

            // -------------------------------------------------------
            // Forbid colors that would destroy the results for this row.
            // for each column that this row used
            hes_c_used.begin(j1);
            j2 = hes_c_used.next_element();
            while (j2 != hes_c_used.end()) {
                // For each row that is non-zero for this column
                // (the used rows have already been checked above).
                hes_not_used.begin(j2);
                j11 = hes_not_used.next_element();
                while (j11 != hes_not_used.end()) {
                    // if this is not the same row, forbid its color
                    if (j11 < j1)
                        forbidden[ color[j11] ] = true;
                    j11 = hes_not_used.next_element();
                }
                j2 = hes_c_used.next_element();
            }


            // pick the color with smallest index
            c = 0;
            while (forbidden[c]) {
                c++;
                CPPAD_ASSERT_UNKNOWN(c <= j1);
            }
            color[j1] = c;
        }
    }


    size_t n_color = 1;
    for (j1 = 0; j1 < n; j1++)
        n_color = std::max(n_color, color[j1] + 1);

    return n_color;
}

/**
 * Compute user specified subset of a sparse Jacobian and a sparse Hessian.
 * 
 * The C++ source code corresponding to this operation is
 * @verbatim
 *    SparseForJacHessian(x, w, y, jac_p, jac_row, jac_col, jac, hes_p, hes_row, hes_col, hes, work)
 * @endverbatim
 * 
 * @tparam Base         is the base type for the recording that is stored in
 *                      this ADFun<Base> object.
 * @tparam VectorBase   is a simple vector class with elements of type @a Base.
 * @tparam VectorSet    is a simple vector class with elements of type @c bool
 *                      or @c std::set<size_t>.
 * @tparam VectorSize   is a simple vector class with elements of type @c size_t.
 * 
 * @param x  is a vector specifying the point at which to compute the Hessian.
 * @param w  is the weighting vector that defines a scalar valued function
 *           by a weighted sum of the components of the vector valued 
 *           function $latex F(x)$$.
 * @param y  is a vector of the dependent variable values.
 * @param jac_p  is the sparsity pattern for the Jacobian that we are calculating.
 * @param jac_row is the vector of row indices for the returned Jacobian values.
 * @param jac_col is the vector of columns indices for the returned 
 *                Jacobian values. It must have the same size are r.
 * @param jac  is the vector of Jacobian values. It must have the same size
 *             are r.  The return value <code>jac[k]</code> is the partial 
 *             of the <code>row[k]</code> component of the function with 
 *             respect the the <code>col[k]</code> of its argument.
 * @param hes_p is the sparsity pattern for the Hessian that we are calculating.
 * @param hes_row is the vector of row indices for the returned Hessian values.
 * @param hes_col is the vector of columns indices for the returned Hessian values.
 *                It must have the same size are r.
 * @param hes is the vector of Hessian values. It must have the same size 
 *            as r. The return value <code>hes[k]</code> is the second 
 *            partial of \f$ w^{\rm T} F(x)\f$ with respect to the
 *            <code>row[k]</code> and <code>col[k]</code> component of 
 *            \f$ x\f$.
 * @param work contains information that depends on the function object, 
 *             sparsity pattern, @c jac_row, @c jac_col, @c hes_row, and
 *             @c hes_col vector. If these values are the same, @c work does
 *             not need to be recomputed.
 * @return Is the number of first order forward and second order reverse
 *         sweeps used to compute the requested values. The total work, not
 *         counting the zero order forward sweep, or the time to combine 
 *         computations, is proportional to this return value.
 */
template<class Base, class VectorBase, class VectorSet, class VectorSize>
size_t sparseForJacHessian(ADFun<Base>& fun,
                           const VectorBase& x,
                           const VectorBase& w,
                           VectorBase& y,
                           const VectorSet& jac_p,
                           const VectorSize& jac_row,
                           const VectorSize& jac_col,
                           VectorBase& jac,
                           const VectorSet& hes_p,
                           const VectorSize& hes_row,
                           const VectorSize& hes_col,
                           VectorBase& hes,
                           SparseForjacHessianWork& work) {
    std::vector<VectorBase> vw(1);
    std::vector<VectorBase> vhes(1);
    vw[0] = w;
    vhes[0] = hes;

    size_t n_sweep = sparseForJacHessian(fun,
                                         x, vw,
                                         y,
                                         jac_p, jac_row, jac_col, jac,
                                         hes_p, hes_row, hes_col, vhes,
                                         work);

    hes = vhes[0];

    return n_sweep;
}

template<class Base, class VectorBase, class VectorVectorBase, class VectorSet, class VectorSize>
size_t sparseForJacHessian(ADFun<Base>& fun,
                           const VectorBase& x,
                           const VectorVectorBase& w,
                           VectorBase& y,
                           const VectorSet& jac_p,
                           const VectorSize& jac_row,
                           const VectorSize& jac_col,
                           VectorBase& jac,
                           const VectorSet& hes_p,
                           const VectorSize& hes_row,
                           const VectorSize& hes_col,
                           VectorVectorBase& hes,
                           SparseForjacHessianWork& work) {
    using CppAD::vectorBool;
    size_t j1, k, c;

    size_t n = fun.Domain();
    size_t m = fun.Range();

    size_t nH = size_t(hes.size());
    size_t jac_K = size_t(jac_row.size());
    size_t hes_K = size_t(hes_row.size());

    CPPADCG_ASSERT_KNOWN(size_t(x.size()) == n,
                         "sparseForJacHessian: size of x not equal domain dimension for f.");

    CPPADCG_ASSERT_KNOWN(size_t(w.size()) == nH,
                         "sparseForJacHessian: size of w not equal to the size of hes.");

    const std::vector<size_t>& jac_scol = work.jac.sort_col;
    const std::vector<size_t>& hes_srow = work.hes.r_sort;
    const std::vector<size_t>& hes_scol = work.hes.c_sort;
    const std::vector<size_t>& hes_user_k = work.hes.k_sort;
    const std::vector<size_t>& color = work.color;

    // some values
    const Base zero(0);
    const Base one(1);

    // check VectorBase is Simple Vector class with Base type elements
    CheckSimpleVector<Base, VectorBase>();

    CPPAD_ASSERT_UNKNOWN(size_t(x.size()) == n);

    work.prepare(fun, jac_row, jac_col, hes_row, hes_col);

    /**
     * coloring
     */
    size_t n_color = colorForwardJacobianHessian(fun, jac_p, hes_p, work);


    // Point at which we are evaluating the Hessian
    y = fun.Forward(0, x);

    // direction vector for calls to forward (columns of jacobian and rows of the Hessian)
    VectorBase u(n);

    // location for return values from forward
    VectorBase dy(m);

    // location for return values from reverse (columns of the Hessian)
    VectorBase ddw(2 * n);

    // initialize the return value
    for (k = 0; k < jac_K; k++)
        jac[k] = zero;
    for (size_t h = 0; h < nH; h++) {
        VectorBase& hesh = hes[h];
        for (k = 0; k < hes_K; k++)
            hesh[k] = zero;
    }

    // loop over colors
    size_t n_sweep = 0;
    for (c = 0; c < n_color; c++) {

        bool anyJac = false;
        size_t kJac = 0;
        for (j1 = 0; j1 < n; j1++) {
            if (color[j1] == c) {
                // find first k such that col[sort_col[k]] has color c
                while (work.jac.user_col[jac_scol[kJac]] < j1)
                    kJac++;
                anyJac = work.jac.user_col[jac_scol[kJac]] == j1;
                if (anyJac)
                    break;
            }
        }

        bool anyHes = false;
        size_t kHessStart = 0;
        for (j1 = 0; j1 < n; j1++) {
            if (color[j1] == c) {
                // find first k such that row[k] has color c
                while (hes_srow[kHessStart] < j1)
                    kHessStart++;
                anyHes = hes_srow[kHessStart] == j1;
                if (anyHes)
                    break;
            }
        }

        if (anyJac || anyHes) {
            n_sweep++;
            // combine all rows with this color
            for (j1 = 0; j1 < n; j1++) {
                u[j1] = zero;
                if (color[j1] == c)
                    u[j1] = one;
            }
            // call forward mode for all these rows at once
            dy = fun.Forward(1, u);

            if (anyJac) {
                // set the corresponding components of the result
                for (j1 = 0; j1 < n; j1++) {
                    if (color[j1] == c) {
                        // find first index in c for this jacobian column
                        while (work.jac.user_col[jac_scol[kJac]] < j1)
                            kJac++;
                        // extract the row results for this column
                        while (work.jac.user_col[jac_scol[kJac]] == j1) {
                            jac[ jac_scol[kJac] ] = dy[ work.jac.user_row[jac_scol[kJac]] ];
                            kJac++;
                        }
                    }
                }
            }

            if (anyHes) {
                n_sweep++;

                for (size_t h = 0; h < nH; h++) {
                    // evaluate derivative of w^T * F'(x) * u
                    ddw = fun.Reverse(2, w[h]);

                    VectorBase& hesh = hes[h];

                    // set the corresponding components of the result
                    size_t kHess = kHessStart;
                    for (j1 = 0; j1 < n; j1++) {
                        if (color[j1] == c) {
                            // find first index in c for this column
                            while (hes_srow[kHess] < j1)
                                kHess++;
                            // extract the results for this row
                            while (hes_srow[kHess] == j1) {
                                size_t j2 = hes_scol[kHess];
                                hesh[ hes_user_k[kHess] ] = ddw[ j2 * 2 + 1 ];
                                kHess++;
                            }
                        }
                    }
                }
            }
        }
    }
    return n_sweep;
}

}
}

#endif
