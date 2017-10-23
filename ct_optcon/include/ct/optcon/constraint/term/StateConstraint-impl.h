/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
 * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::StateConstraint(const state_vector_t& xLow,
    const state_vector_t& xHigh)
{
    Base::lb_.resize(STATE_DIM);
    Base::ub_.resize(STATE_DIM);
    // The terminal state constraint is treated as equality constraint, therefore, ub = lb
    Base::lb_ = xLow;
    Base::ub_ = xHigh;
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
    return STATE_DIM;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::evaluate(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return x;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateCppadCg(
    const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
    const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
    ct::core::ADCGScalar t)
{
    return x;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianState(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM>::Identity();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInput(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM>::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::getNumNonZerosJacobianState() const
{
    return STATE_DIM;
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
    return state_vector_t::Ones();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternState(VectorXi& rows, VectorXi& cols)
{
    this->genDiagonalIndices(STATE_DIM, rows, cols);
}

}  // namespace optcon
}  // namespace ct
