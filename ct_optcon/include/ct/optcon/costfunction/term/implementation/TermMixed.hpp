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

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR>::TermMixed(
		const control_state_matrix_t& P) :
		P_(P)
{
	x_ref_.setZero();	// default values
	u_ref_.setZero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR>::TermMixed() {
	P_.setZero();	// default values
	x_ref_.setZero();
	u_ref_.setZero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR>::TermMixed(
		const control_state_matrix_t& P,
		const core::StateVector<STATE_DIM, SCALAR>& x_ref,
		core::ControlVector<CONTROL_DIM, SCALAR>& u_ref):
		P_(P),
		x_ref_(x_ref),
		u_ref_(u_ref)
{}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR>::TermMixed(const TermMixed<STATE_DIM, CONTROL_DIM, SCALAR>& arg):
	TermBase<STATE_DIM, CONTROL_DIM, SCALAR>(arg),
	P_(arg.P_),
	x_ref_(arg.x_ref_),
	u_ref_(arg.u_ref_)
{}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR>* TermMixed<STATE_DIM, CONTROL_DIM, SCALAR>::clone () const {
		return new TermMixed(*this);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void TermMixed<STATE_DIM, CONTROL_DIM, SCALAR>::setWeights(const control_state_matrix_double_t& P)
{
    P_ = P.template cast<SCALAR>();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void TermMixed<STATE_DIM, CONTROL_DIM, SCALAR>::setStateAndControlReference(
		const core::StateVector<STATE_DIM>& x_ref,
		const core::ControlVector<CONTROL_DIM>& u_ref)
{
	x_ref_ = x_ref.template cast<SCALAR>();
	u_ref_ = u_ref.template cast<SCALAR>();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SCALAR TermMixed<STATE_DIM, CONTROL_DIM, SCALAR>::evaluate(
		const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x,
		const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> &u,
		const SCALAR& t)
{
    Eigen::Matrix<SCALAR, STATE_DIM, 1> xDiff = (x-x_ref_.template cast<SCALAR>());
    Eigen::Matrix<SCALAR, CONTROL_DIM, 1> uDiff = (u-u_ref_.template cast<SCALAR>());

    return (uDiff.transpose() * P_.template cast<SCALAR>() * xDiff)(0,0);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ct::core::StateVector<STATE_DIM, SCALAR> TermMixed<STATE_DIM, CONTROL_DIM, SCALAR>::stateDerivative(
		const ct::core::StateVector<STATE_DIM, SCALAR> &x,
		const ct::core::ControlVector<CONTROL_DIM, SCALAR> &u,
		const SCALAR& t)
{
    core::ControlVector<CONTROL_DIM, SCALAR> uDiff = (u-u_ref_);

    return  P_.transpose() * uDiff;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename TermMixed<STATE_DIM, CONTROL_DIM, SCALAR>::state_matrix_t TermMixed<STATE_DIM, CONTROL_DIM, SCALAR>::stateSecondDerivative(
		const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t)
{
	return state_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
core::ControlVector<CONTROL_DIM, SCALAR> TermMixed<STATE_DIM, CONTROL_DIM, SCALAR>::controlDerivative(
		const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t)
{
    core::StateVector<STATE_DIM, SCALAR> xDiff = (x-x_ref_);

    return P_ * xDiff;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename TermMixed<STATE_DIM, CONTROL_DIM, SCALAR>::control_matrix_t TermMixed<STATE_DIM, CONTROL_DIM, SCALAR>::controlSecondDerivative(
		const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t)
{
	return control_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename TermMixed<STATE_DIM, CONTROL_DIM, SCALAR>::control_state_matrix_t TermMixed<STATE_DIM, CONTROL_DIM, SCALAR>::stateControlDerivative(
		const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t)
{
	return P_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void TermMixed<STATE_DIM, CONTROL_DIM, SCALAR>::loadConfigFile(const std::string& filename, const std::string& termName, bool verbose)
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
