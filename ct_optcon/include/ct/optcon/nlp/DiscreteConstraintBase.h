/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <Eigen/Core>

namespace ct {
namespace optcon {
namespace tpl {

/**
 * @ingroup    DMS
 *
 * @brief      Implements an abstract base class from which all the discrete
 *             custom NLP constraints should derive
 */
template <typename SCALAR>
class DiscreteConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using VectorXs = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;

    /**
     * @brief      Default constructor
     */
    DiscreteConstraintBase() = default;
    /**
     * @brief      Destructor
     */
    virtual ~DiscreteConstraintBase() = default;
    /**
     * @brief      Evaluates the constraint violation
     *
     * @return     A vector of the evaluated constraint violation
     */
    virtual VectorXs eval() = 0;

    /**
     * @brief      Returns the non zero elements of the eval method with respect
     *             to the optimization variables
     *
     * @return     A vector of the non zero elements of the constraint jacobian
     */
    virtual VectorXs evalSparseJacobian() = 0;

    /**
     * @brief      Returns size of the constraint vector
     *
     * @return     The size of the constraint vector (should be equal to the
     *             size of the return value of the eval method)
     */
    virtual size_t getConstraintSize() = 0;

    /**
     * @brief      Returns the number of non zero elements of the jacobian
     *
     * @return     The number of non zero elements of the jacobian (which should
     *             be equal to the return value of evalSparseJacobian)
     */
    virtual size_t getNumNonZerosJacobian() = 0;

    /**
     * @brief      Returns the sparsity structure of the constraint jacobian
     *
     * @param[out]      iRow_vec  A vector containing the row indices of the non zero
     *                       elements of the constraint jacobian
     * @param[out]      jCol_vec  A vector containing the column indices of the non
     *                       zero elements of the constraint jacobian
     */
    virtual void genSparsityPattern(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) = 0;


    /**
     * @brief      Returns the sparsity structure of the constraint hessian
     *
     * @param[out]      iRow_vec  A vector containing the row indices of the non zero
     *                       elements of the constraint hessian
     * @param[out]      jCol_vec  A vector containing the column indices of the non
     *                       zero elements of the constraint hessian
     */
    virtual void genSparsityPatternHessian(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec)
    {
        throw std::runtime_error(
            "genSparsityPatternHessian() for DiscreteConstraintBase not implemented. Use Hessian approximation.");
    }

    /**
     * @brief      Returns the non zero elements (values) of the Hessian matrix of this constraint
     *
     * @param[in] optVec 	A vector holding the current optimization variable vector
     *
     * @param[in] lambda 	A vector holding the current constraint multipliers (one scalar for each row of constraints)
     *
     * @param[out] sparseHes	The sparse hessian values
     */
    virtual void sparseHessianValues(const Eigen::VectorXd& optVec,
        const Eigen::VectorXd& lambda,
        Eigen::VectorXd& sparseHes)
    {
        throw std::runtime_error(
            "sparseHessianValues() for DiscreteConstraintBase not implemented. Use Hessian approximation.");
    }


    /**
     * @brief      Returns the lower bound of the constraint
     *
     * @return     The lower constraint bound
     */
    virtual VectorXs getLowerBound() = 0;

    /**
     * @brief      Returns the upper bound of the constraint
     *
     * @return     The upper constraint bound
     */
    virtual VectorXs getUpperBound() = 0;

    Eigen::VectorXi& iRowHessian() { return iRowHessian_; }
    Eigen::VectorXi& jColHessian() { return jColHessian_; }
protected:
    /**
     * @brief      This method generates Row and Column vectors which indicate
     *             the sparsity pattern of the constraint jacobian for a
     *             quadratic matrix block containing diagonal entries only
     *
     * @param[in]  col_start     The starting column of the jCol vec
     * @param[in]  num_elements  The size of the matrix block
     * @param[out] iRow_vec      The resulting row vector
     * @param[out] jCol_vec      The resuling column vector
     * @param[in]  indexNumber   The starting inserting index for iRow and jCol
     *
     * @return     indexnumber plus num_elements
     */
    size_t genDiagonalIndices(const size_t col_start,
        const size_t num_elements,
        Eigen::VectorXi& iRow_vec,
        Eigen::VectorXi& jCol_vec,
        const size_t indexNumber);

    /**
     * @brief      This method generates Row and Column vectors which indicate
     *             the sparsity pattern of the constraint jacobian for an
     *             arbitrary dense matrix block
     *
     * @param[in]  col_start    The starting column of the jCol vec
     * @param[in]  num_rows     The number of rows of the matrix block
     * @param[in]  num_cols     The number of columns of the matrix block
     * @param[out] iRow_vec     The resulting row vector
     * @param[out] jCol_vec     The resuling column vector
     * @param[in]  indexNumber  The starting inserting index for iRow and jCol
     *
     * @return     The indexnumber plus the number of elements contained in the
     *             matrix block
     */
    size_t genBlockIndices(const size_t col_start,
        const size_t num_rows,
        const size_t num_cols,
        Eigen::VectorXi& iRow_vec,
        Eigen::VectorXi& jCol_vec,
        const size_t indexNumber);

    Eigen::VectorXi iRowHessian_;
    Eigen::VectorXi jColHessian_;
};

template <typename SCALAR>
inline size_t DiscreteConstraintBase<SCALAR>::genDiagonalIndices(const size_t col_start,
    const size_t num_elements,
    Eigen::VectorXi& iRow_vec,
    Eigen::VectorXi& jCol_vec,
    const size_t indexNumber)
{
    Eigen::VectorXi new_row_indices;
    Eigen::VectorXi new_col_indices;
    new_row_indices.resize(num_elements);
    new_col_indices.resize(num_elements);

    size_t count = 0;

    for (size_t i = 0; i < num_elements; ++i)
    {
        new_row_indices(count) = i;
        new_col_indices(count) = col_start + i;
        count++;
    }

    assert(count == num_elements);

    iRow_vec.segment(indexNumber, num_elements) = new_row_indices;
    jCol_vec.segment(indexNumber, num_elements) = new_col_indices;

    return count;
}

template <typename SCALAR>
inline size_t DiscreteConstraintBase<SCALAR>::genBlockIndices(const size_t col_start,
    const size_t num_rows,
    const size_t num_cols,
    Eigen::VectorXi& iRow_vec,
    Eigen::VectorXi& jCol_vec,
    const size_t indexNumber)
{
    size_t num_gen_indices = num_rows * num_cols;

    Eigen::VectorXi new_row_indices;
    Eigen::VectorXi new_col_indices;
    new_row_indices.resize(num_gen_indices);
    new_col_indices.resize(num_gen_indices);

    size_t count = 0;

    for (size_t row = 0; row < num_rows; ++row)
    {
        for (size_t col = col_start; col < col_start + num_cols; ++col)
        {
            new_row_indices(count) = row;
            new_col_indices(count) = col;
            count++;
        }
    }

    assert(count == num_gen_indices);

    iRow_vec.segment(indexNumber, num_gen_indices) = new_row_indices;
    jCol_vec.segment(indexNumber, num_gen_indices) = new_col_indices;

    return num_gen_indices;
}
}

using DiscreteConstraintBase = tpl::DiscreteConstraintBase<double>;

}  // namespace optcon
}  // namespace ct
