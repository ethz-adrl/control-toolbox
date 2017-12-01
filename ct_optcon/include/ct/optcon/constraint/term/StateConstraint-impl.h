/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::StateConstraint(const state_vector_t& xLow,
    const state_vector_t& xHigh)
{
    for (size_t i = 0; i < STATE_DIM; i++)
    {
        if (xLow(i) > xHigh(i))
            throw std::runtime_error("StateConstraint: wrong boundaries: xLow > xHigh");
    }

    constrSize_ = STATE_DIM;
    Base::lb_.resize(STATE_DIM);
    Base::ub_.resize(STATE_DIM);
    sparsity_ = Eigen::Matrix<int, STATE_DIM, 1>::Ones();  // dense sparsity!
    sparsity_J_.resize(STATE_DIM, STATE_DIM);
    sparsity_J_.setIdentity();
    Base::lb_ = xLow;
    Base::ub_ = xHigh;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::StateConstraint(const VectorXs& lb,
    const VectorXs& ub,
    const Eigen::VectorXi& state_sparsity)
{
    // make sure the sparsity pattern is correct and consists only of ones and zeros
    assert(state_sparsity.maxCoeff() <= 1);
    assert(state_sparsity.minCoeff() >= 0);

    constrSize_ = (size_t)state_sparsity.sum();

    if (lb.rows() != constrSize_ | ub.rows() != constrSize_)
        throw std::runtime_error("StateConstraint: wrong constraint sizes in StateConstraint");

    for (size_t i = 0; i < constrSize_; i++)
    {
        if (lb(i) > ub(i))
            throw std::runtime_error("State constraint wrong boundaries: lb > ub");
    }

    Base::lb_.resize(constrSize_);
    Base::ub_.resize(constrSize_);
    sparsity_ = state_sparsity;
    sparsity_J_.resize(constrSize_, STATE_DIM);
    sparsity_J_ = diagSparsityVecToSparsityMat(state_sparsity, constrSize_);
    Base::lb_ = lb;
    Base::ub_ = ub;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::StateConstraint(const StateConstraint& arg)
    : Base(arg), sparsity_(arg.sparsity_), sparsity_J_(arg.sparsity_J_), constrSize_(arg.constrSize_)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::~StateConstraint()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>* StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::clone() const
{
    return new StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::getConstraintSize() const
{
    return constrSize_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::evaluate(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return sparsity_J_ * x;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateCppadCg(
    const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
    const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
    ct::core::ADCGScalar t)
{
    return sparsity_J_.template cast<ct::core::ADCGScalar>() * x;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianState(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return sparsity_J_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInput(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
	MatrixXs jac (constrSize_, CONTROL_DIM);
	jac.setZero();
    return jac;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::getNumNonZerosJacobianState() const
{
    return constrSize_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::getNumNonZerosJacobianInput() const
{
    return 0;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateSparse(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
	Eigen::Matrix<SCALAR, -1, 1> jac (constrSize_, 1);
	jac.setConstant(1.0);
    return jac;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternState(VectorXi& rows, VectorXi& cols)
{
    this->genSparseDiagonalIndices(sparsity_, rows, cols);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::sparsity_matrix_t
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::diagSparsityVecToSparsityMat(const VectorXi& spVec, const size_t& nConstr)
{
    sparsity_matrix_t mat(nConstr, STATE_DIM);
    mat.setZero();

    size_t count = 0;
    assert(spVec.rows() == STATE_DIM);
    for (size_t i = 0; i < STATE_DIM; i++)
    {
        if (spVec(i) == 1)
        {
            mat(count, i) = 1;
            count++;
        }
    }
    return mat;
}


}  // namespace optcon
}  // namespace ct
