/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#ifdef CPPAD

#include <cppad/cppad.hpp>
#include <Eigen/Core>

namespace ct {
namespace core {
namespace internal {

//! Convenience class to handle sparsity patterns, e.g. in Jacobians
class SparsityPattern
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SparsityPattern() = default;

    virtual ~SparsityPattern() = default;

    //! initialize the pattern
    /*!
	 * Initializes the pattern given a binary matrix. A "true" entry in the
	 * matrix means that there is a non-zero entry in the Jacobian.
	 * @param sparsity The sparsity pattern
	 * @tparam ROWS number of rows in matrix
	 * @tparam COLS number of columns in matrix
	 */
    template <int ROWS, int COLS>
    void initPattern(const Eigen::Matrix<bool, ROWS, COLS>& sparsity)
    {
        clearWork();

        const size_t rows = sparsity.rows();
        const size_t cols = sparsity.cols();

        sparsity_.resize(rows * cols);

        for (size_t i = 0; i < rows * cols; i++)
            sparsity_[i] = sparsity.array()(i);

        size_t jacEntries = sparsity.template cast<int>().sum();

        row_.resize(jacEntries);
        col_.resize(jacEntries);
        size_t count = 0;
        for (size_t i = 0; i < rows; i++)
        {
            for (size_t j = 0; j < cols; j++)
            {
                if (sparsity(i, j))
                {
                    row_[count] = j;
                    col_[count] = i;
                    count++;
                }
            }
        }
    }

    //! returns the sparsity pattern as a vector (row-major)
    /*!
	 * @return sparsity pattern vector
	 */
    const CppAD::vector<bool>& sparsity() const { return sparsity_; }
    //! returns the row indices of a row-column sparsity representation
    /*!
	 * In a row-column representation, only the non-zero entries get saved.
	 * This method returns the row indices of the row-column representation. Since there
	 * can be several non-zero entries in the same row, the same row index
	 * can appear repeatedly in this vector.
	 * @return row indeces of row-column representation
	 */
    const CppAD::vector<size_t>& row() const { return row_; }
    //! returns the column indices of a row-column sparsity representation
    /*!
	 * In a row-column representation, only the non-zero entries get saved.
	 * This method returns the column indices of the row-column representation. Since there
	 * can be several non-zero entries in the same column, the same column index
	 * can appear repeatedly in this vector.
	 * @return column indeces of row-column representation
	 */
    const CppAD::vector<size_t>& col() const { return col_; }
    //! work area for CppAD
    /*!
	 * CppAD can make use of previous results from a sparsity analysis.
	 * This method returns the "work" area of CppAD sparsity
	 * @return CppAD sparsity work
	 */
    CppAD::sparse_jacobian_work& workJacobian() { return workJacobian_; }
    CppAD::sparse_hessian_work& workHessian() { return workHessian_; }
    /**
	 * @brief      Clears the cppad internal work done on the sparsity pattern
	 */
    void clearWork()
    {
        workJacobian_.clear();
        workHessian_.clear();
    }

private:
    CppAD::vector<bool> sparsity_;              //! sparsity in vector representation
    CppAD::vector<size_t> row_;                 //! sparsity row indices for row-column representation
    CppAD::vector<size_t> col_;                 //! sparsity column indices for row-column representation
    CppAD::sparse_jacobian_work workJacobian_;  //! CppAD internal "work"
    CppAD::sparse_hessian_work workHessian_;
};


} /* namespace internal */
} /* namespace core */
} /* namespace ct */

#endif
