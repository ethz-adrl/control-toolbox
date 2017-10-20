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

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermMixed(
		const control_state_matrix_t& P) :
		P_(P)
{
	x_ref_.setZero();	// default values
	u_ref_.setZero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermMixed() {
	P_.setZero();	// default values
	x_ref_.setZero();
	u_ref_.setZero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermMixed(
		const control_state_matrix_t& P,
		const core::StateVector<STATE_DIM, SCALAR_EVAL>& x_ref,
		core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u_ref):
		P_(P),
		x_ref_(x_ref),
		u_ref_(u_ref)
{}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermMixed(const TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>& arg):
	TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>(arg),
	P_(arg.P_),
	x_ref_(arg.x_ref_),
	u_ref_(arg.u_ref_)
{}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>* TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::clone () const {
		return new TermMixed(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::~TermMixed()
{}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::setWeights(const control_state_matrix_double_t& P)
{
    P_ = P.template cast<SCALAR_EVAL>();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::setStateAndControlReference(
		const core::StateVector<STATE_DIM>& x_ref,
		const core::ControlVector<CONTROL_DIM>& u_ref)
{
	x_ref_ = x_ref.template cast<SCALAR_EVAL>();
	u_ref_ = u_ref.template cast<SCALAR_EVAL>();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
template<typename SC>
SC TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::evalLocal(
		const Eigen::Matrix<SC, STATE_DIM, 1> &x, const Eigen::Matrix<SC, CONTROL_DIM, 1> &u, const SC& t)
{
    Eigen::Matrix<SC, STATE_DIM, 1> xDiff = (x-x_ref_.template cast<SC>());
    Eigen::Matrix<SC, CONTROL_DIM, 1> uDiff = (u-u_ref_.template cast<SC>());

    return (uDiff.transpose() * P_.template cast<SC>() * xDiff)(0,0);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
SCALAR TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::evaluate(
		const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x,
		const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> &u,
		const SCALAR& t)
{
	return evalLocal<SCALAR>(x, u, t);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
ct::core::ADCGScalar TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::evaluateCppadCg(
	const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
	const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
	ct::core::ADCGScalar t)
	{
	return evalLocal<ct::core::ADCGScalar>(x, u, t);
	}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
ct::core::StateVector<STATE_DIM, SCALAR_EVAL> TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateDerivative(
		const ct::core::StateVector<STATE_DIM, SCALAR_EVAL> &x,
		const ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> &u,
		const SCALAR_EVAL& t)
{
    core::ControlVector<CONTROL_DIM, SCALAR_EVAL> uDiff = (u-u_ref_);

    return  P_.transpose() * uDiff;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::state_matrix_t TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateSecondDerivative(
		const core::StateVector<STATE_DIM, SCALAR_EVAL> &x, const core::ControlVector<CONTROL_DIM, SCALAR_EVAL> &u, const SCALAR_EVAL& t)
{
	return state_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
core::ControlVector<CONTROL_DIM, SCALAR_EVAL> TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::controlDerivative(
		const core::StateVector<STATE_DIM, SCALAR_EVAL> &x, const core::ControlVector<CONTROL_DIM, SCALAR_EVAL> &u, const SCALAR_EVAL& t)
{
    core::StateVector<STATE_DIM, SCALAR_EVAL> xDiff = (x-x_ref_);

    return P_ * xDiff;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::control_matrix_t TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::controlSecondDerivative(
		const core::StateVector<STATE_DIM, SCALAR_EVAL> &x, const core::ControlVector<CONTROL_DIM, SCALAR_EVAL> &u, const SCALAR_EVAL& t)
{
	return control_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::control_state_matrix_t TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateControlDerivative(
		const core::StateVector<STATE_DIM, SCALAR_EVAL> &x, const core::ControlVector<CONTROL_DIM, SCALAR_EVAL> &u, const SCALAR_EVAL& t)
{
	return P_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::loadConfigFile(const std::string& filename, const std::string& termName, bool verbose)
{
       loadMatrixCF(filename,"P", P_,termName);
       loadMatrixCF(filename,"x_des", x_ref_,termName);
       loadMatrixCF(filename,"u_des", u_ref_,termName);
       if (verbose){
		   std::cout<<"Read P as P = \n"<<P_<<std::endl;
		   std::cout<<"Read x_ref as x_ref = \n"<<x_ref_.transpose()<<std::endl;
		   std::cout<<"Read u_ref as u_ref = \n"<<u_ref_.transpose()<<std::endl;
       }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::updateReferenceState (const Eigen::Matrix<SCALAR_EVAL, STATE_DIM, 1>& newRefState)
{
	x_ref_ = newRefState;
}

}
}
