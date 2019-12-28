/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::ConstraintBase(std::string name) : name_(name)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::ConstraintBase(const ConstraintBase& arg)
    : lb_(arg.lb_), ub_(arg.ub_), name_(arg.name_)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::~ConstraintBase()
{
}

#ifdef CPPADCG
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateCppadCg(
    const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
    const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
    ct::core::ADCGScalar t)
{
    throw std::runtime_error("Term " + name_ + " has no Implementation of evaluateCppaCg.");
}
#endif

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianState(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    throw std::runtime_error(
        "This constraint function element is not implemented for the given term."
        "Please use either auto-diff cost function or implement the analytical derivatives manually.");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInput(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    throw std::runtime_error(
        "This constraint function element is not implemented for the given term."
        "Please use either auto-diff cost function or implement the analytical derivatives manually.");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::getLowerBound() const
{
    return lb_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::getUpperBound() const
{
    return ub_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::getName(std::string& constraintName) const
{
    constraintName = name_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::setName(const std::string constraintName)
{
    name_ = constraintName;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::getNumNonZerosJacobianState() const
{
    return STATE_DIM * getConstraintSize();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::getNumNonZerosJacobianInput() const
{
    return CONTROL_DIM * getConstraintSize();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateSparse(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    MatrixXs jacState = jacobianState(x, u, t);

    VectorXs jac(Eigen::Map<VectorXs>(jacState.data(), jacState.rows() * jacState.cols()));

    return jac;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputSparse(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    MatrixXs jacInput = jacobianInput(x, u, t);

    VectorXs jac(Eigen::Map<VectorXs>(jacInput.data(), jacInput.rows() * jacInput.cols()));
    return jac;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternState(Eigen::VectorXi& rows, Eigen::VectorXi& cols)
{
    genBlockIndices(getConstraintSize(), STATE_DIM, rows, cols);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternInput(Eigen::VectorXi& rows, Eigen::VectorXi& cols)
{
    genBlockIndices(getConstraintSize(), CONTROL_DIM, rows, cols);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::genDiagonalIndices(const size_t num_elements,
    Eigen::VectorXi& iRow_vec,
    Eigen::VectorXi& jCol_vec)
{
    iRow_vec.resize(num_elements);
    jCol_vec.resize(num_elements);

    size_t count = 0;

    for (size_t i = 0; i < num_elements; ++i)
    {
        iRow_vec(count) = i;
        jCol_vec(count) = i;
        count++;
    }
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::genSparseDiagonalIndices(const Eigen::VectorXi& diag_sparsity,
    Eigen::VectorXi& iRow_vec,
    Eigen::VectorXi& jCol_vec)
{
    // make sure the sparsity pattern is correct and consists only of ones and zeros
    assert(diag_sparsity.maxCoeff() <= 1);
    assert(diag_sparsity.minCoeff() >= 0);

    const int num_elements = diag_sparsity.sum();

    iRow_vec.resize(num_elements);
    jCol_vec.resize(num_elements);

    size_t count = 0;

    for (int i = 0; i < diag_sparsity.rows(); ++i)
    {
        if (diag_sparsity(i) == 1)
        {
            iRow_vec(count) = i;
            jCol_vec(count) = i;
            count++;
        }
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>::genBlockIndices(const size_t num_rows,
    const size_t num_cols,
    Eigen::VectorXi& iRow_vec,
    Eigen::VectorXi& jCol_vec)
{
    size_t num_gen_indices = num_rows * num_cols;

    iRow_vec.resize(num_gen_indices);
    jCol_vec.resize(num_gen_indices);

    size_t count = 0;

    for (size_t row = 0; row < num_rows; ++row)
    {
        for (size_t col = 0; col < num_cols; ++col)
        {
            iRow_vec(count) = row;
            jCol_vec(count) = col;
            count++;
        }
    }
}


}  // namespace optcon
}  // namespace ct
