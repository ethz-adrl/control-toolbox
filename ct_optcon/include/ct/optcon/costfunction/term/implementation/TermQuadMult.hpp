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
TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR>::TermQuadMult(
        const state_matrix_t& Q,
        const control_matrix_t& R) :
        Q_(Q),
        R_(R)
{
    x_ref_.setZero();   // default values
    u_ref_.setZero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR>::TermQuadMult() {
    Q_.setIdentity();   // default values
    R_.setIdentity();
    x_ref_.setZero();
    u_ref_.setZero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR>::TermQuadMult(
        const state_matrix_t& Q,
        const control_matrix_t& R,
        const core::StateVector<STATE_DIM, SCALAR>& x_ref,
        core::ControlVector<CONTROL_DIM, SCALAR>& u_ref):
        Q_(Q),
        R_(R),
        x_ref_(x_ref),
        u_ref_(u_ref)
{}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR>::TermQuadMult(const TermQuadMult& arg):
    TermBase<STATE_DIM, CONTROL_DIM, SCALAR>(arg),
    Q_(arg.Q_),
    R_(arg.R_),
    x_ref_(arg.x_ref_),
    u_ref_(arg.u_ref_)
{}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR>* TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR>::clone () const {
    return new TermQuadMult(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR>::~TermQuadMult() {}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR>::setWeights(const state_matrix_double_t& Q, const control_matrix_double_t& R)
{
    Q_ = Q.template cast<SCALAR>();
    R_ = R.template cast<SCALAR>();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR>::setStateAndControlReference(
        const core::StateVector<STATE_DIM>& x_ref,
        core::ControlVector<CONTROL_DIM>& u_ref)
{
    x_ref_ = x_ref.template cast<SCALAR>();;
    u_ref_ = u_ref.template cast<SCALAR>();;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SCALAR TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR>::evaluate(
        const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x,
        const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> &u,
        const SCALAR& t)
{
    Eigen::Matrix<SCALAR, STATE_DIM, 1> xDiff = (x-x_ref_.template cast<SCALAR>());
    Eigen::Matrix<SCALAR, CONTROL_DIM, 1> uDiff = (u-u_ref_.template cast<SCALAR>());

    return (xDiff.transpose() * Q_.template cast<SCALAR>() * xDiff)(0,0) * (uDiff.transpose() * R_.template cast<SCALAR>() * uDiff)(0,0);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
core::StateVector<STATE_DIM, SCALAR> TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR>::stateDerivative(
        const core::StateVector<STATE_DIM, SCALAR> &x,
        const core::ControlVector<CONTROL_DIM, SCALAR> &u,
        const SCALAR& t)
{
    core::StateVector<STATE_DIM, SCALAR> xDiff = (x-x_ref_);
    core::ControlVector<CONTROL_DIM, SCALAR> uDiff = (u-u_ref_);

    SCALAR r = (uDiff.transpose() * R_ * uDiff)(0,0);

    return (xDiff.transpose() * Q_.transpose() + xDiff.transpose() * Q_)*r;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR>::state_matrix_t TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR>::stateSecondDerivative(
		const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t)
{
    core::ControlVector<CONTROL_DIM, SCALAR> uDiff = (u-u_ref_);

    SCALAR r = (uDiff.transpose() * R_ * uDiff)(0,0);

    return (Q_ + Q_.transpose())*r;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
core::ControlVector<CONTROL_DIM, SCALAR> TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR>::controlDerivative(
		const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t)
{
    core::StateVector<STATE_DIM, SCALAR> xDiff = (x-x_ref_);
    core::ControlVector<CONTROL_DIM, SCALAR> uDiff = (u-u_ref_);

    SCALAR q = (xDiff.transpose() * Q_ * xDiff)(0,0);

    return (uDiff.transpose() * R_.transpose() + uDiff.transpose() * R_)*q;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR>::control_matrix_t TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR>::controlSecondDerivative(
		const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t)
{
    core::StateVector<STATE_DIM, SCALAR> xDiff = (x-x_ref_);

    SCALAR q = (xDiff.transpose() * Q_ * xDiff)(0,0);

    return (R_ + R_.transpose())*q;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR>::control_state_matrix_t TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR>::stateControlDerivative(
		const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t)
{
    core::StateVector<STATE_DIM, SCALAR> xDiff = (x-x_ref_);
    core::ControlVector<CONTROL_DIM, SCALAR> uDiff = (u-u_ref_);

    return (uDiff.transpose() * R_.transpose() + uDiff.transpose() * R_).transpose()*(xDiff.transpose() * Q_.transpose() + xDiff.transpose() * Q_);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR>::loadConfigFile(const std::string& filename, const std::string& termName, bool verbose)
{
       loadMatrixCF(filename,"Q", Q_,termName);
       loadMatrixCF(filename,"R", R_,termName);
       loadMatrixCF(filename,"x_des", x_ref_,termName);
       loadMatrixCF(filename,"u_des", u_ref_,termName);
       if (verbose){
           std::cout<<"Read Q as Q = \n"<<Q_<<std::endl;
           std::cout<<"Read R as R = \n"<<R_<<std::endl;
           std::cout<<"Read x_ref as x_ref = \n"<<x_ref_.transpose()<<std::endl;
           std::cout<<"Read u_ref as u_ref = \n"<<u_ref_.transpose()<<std::endl;
       }
}
