/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t DERIVED_DIM, size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
BoxConstraintBase<DERIVED_DIM, STATE_DIM, CONTROL_DIM, SCALAR>::BoxConstraintBase(const decision_vector_t& lb,
    const decision_vector_t& ub)
{
    // check if box constraints are meaningful
    sanityCheck(DERIVED_DIM, lb, ub);

    // this box constraint is 'dense' (one dense jacobian diagonal, one zero jacobian)
    constrSize_ = DERIVED_DIM;
    Base::lb_.resize(DERIVED_DIM);
    Base::ub_.resize(DERIVED_DIM);
    sparsity_ = Eigen::Matrix<int, DERIVED_DIM, 1>::Ones();
    sparsity_J_.resize(DERIVED_DIM, DERIVED_DIM);
    sparsity_J_.setIdentity();
    Base::lb_ = lb.template cast<SCALAR>();
    Base::ub_ = ub.template cast<SCALAR>();
}

template <size_t DERIVED_DIM, size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
BoxConstraintBase<DERIVED_DIM, STATE_DIM, CONTROL_DIM, SCALAR>::BoxConstraintBase(const VectorXs& lb,
    const VectorXs& ub,
    const Eigen::VectorXi& sparsity_vec)
{
    // make sure the provided sparsity pattern is correct and consists only of ones and zeros
    assert(sparsity_vec.maxCoeff() <= 1);
    assert(sparsity_vec.minCoeff() >= 0);

    constrSize_ = (size_t)sparsity_vec.sum();

    // make sure the provided bounds are consistent
    sanityCheck(constrSize_, lb, ub);

    Base::lb_.resize(constrSize_);
    Base::ub_.resize(constrSize_);
    sparsity_ = sparsity_vec;
    sparsity_J_.resize(constrSize_, DERIVED_DIM);
    sparsity_J_ = diagSparsityVecToSparsityMat(sparsity_vec, constrSize_);
    Base::lb_ = lb.template cast<SCALAR>();
    Base::ub_ = ub.template cast<SCALAR>();
}

template <size_t DERIVED_DIM, size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
BoxConstraintBase<DERIVED_DIM, STATE_DIM, CONTROL_DIM, SCALAR>::BoxConstraintBase(const BoxConstraintBase& arg)
    : Base(arg), sparsity_(arg.sparsity_), sparsity_J_(arg.sparsity_J_), constrSize_(arg.constrSize_)
{
}

template <size_t DERIVED_DIM, size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
BoxConstraintBase<DERIVED_DIM, STATE_DIM, CONTROL_DIM, SCALAR>::~BoxConstraintBase()
{
}

template <size_t DERIVED_DIM, size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t BoxConstraintBase<DERIVED_DIM, STATE_DIM, CONTROL_DIM, SCALAR>::getConstraintSize() const
{
    return constrSize_;
}

template <size_t DERIVED_DIM, size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename BoxConstraintBase<DERIVED_DIM, STATE_DIM, CONTROL_DIM, SCALAR>::sparsity_matrix_t
BoxConstraintBase<DERIVED_DIM, STATE_DIM, CONTROL_DIM, SCALAR>::diagSparsityVecToSparsityMat(const VectorXi& spVec,
    const size_t& nConstr)
{
    // set up sparsity matrix all zero
    sparsity_matrix_t mat(nConstr, DERIVED_DIM);
    mat.setZero();

    // set selected elements to one
    size_t count = 0;
    assert(spVec.rows() == DERIVED_DIM);
    for (size_t i = 0; i < DERIVED_DIM; i++)
    {
        if (spVec(i) == 1)
        {
            mat(count, i) = 1;
            count++;
        }
    }
    return mat;
}


template <size_t DERIVED_DIM, size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void BoxConstraintBase<DERIVED_DIM, STATE_DIM, CONTROL_DIM, SCALAR>::sanityCheck(const size_t& nCon,
    const VectorXs& lb,
    const VectorXs& ub) const
{
    // assert that the size of constraint vectors is equal to the computed/given number of constraints
    if ((lb.rows() != static_cast<int>(nCon)) | (ub.rows() != static_cast<int>(nCon)))
    {
        std::cout << "no. Constraints: " << nCon << std::endl;
        std::cout << "BoxConstraintBase: lb " << lb.transpose() << std::endl;
        std::cout << "BoxConstraintBase: ub " << ub.transpose() << std::endl;
        throw std::runtime_error("BoxConstraintBase: wrong constraint sizes in StateConstraint");
    }

    // assert that the boundaries are meaningful
    for (size_t i = 0; i < nCon; i++)
    {
        if (lb(i) > ub(i))
        {
            std::cout << "BoxConstraintBase: lb " << lb.transpose() << std::endl;
            std::cout << "BoxConstraintBase: ub " << ub.transpose() << std::endl;
            throw std::runtime_error("BoxConstraintBase: wrong boundaries: lb > ub");
        }
    }
}


template <size_t DERIVED_DIM, size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void BoxConstraintBase<DERIVED_DIM, STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternSparseJacobian(
    const VectorXi& sparsity_vec,
    const size_t& constrSize,
    VectorXi& rows,
    VectorXi& cols)
{
    Base::genSparseDiagonalIndices(sparsity_vec, rows, cols);
    for (size_t i = 0; i < constrSize; i++)
        rows(i) = i;
}


}  // namespace optcon
}  // namespace ct
