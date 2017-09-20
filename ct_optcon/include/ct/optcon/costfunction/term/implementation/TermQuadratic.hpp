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
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::TermQuadratic(
		const state_matrix_t& Q,
		const control_matrix_t& R) :
		Q_(Q),
		R_(R)
{
	x_ref_.setZero();	// default values
	u_ref_.setZero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::TermQuadratic() {
	Q_.setIdentity();	// default values
	R_.setIdentity();
	x_ref_.setZero();
	u_ref_.setZero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::TermQuadratic(
		const state_matrix_t& Q,
		const control_matrix_t& R,
		const core::StateVector<STATE_DIM, SCALAR>& x_ref,
		const core::ControlVector<CONTROL_DIM, SCALAR>& u_ref):
		Q_(Q),
		R_(R),
		x_ref_(x_ref),
		u_ref_(u_ref)
{}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::TermQuadratic(const TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>& arg):
	TermBase<STATE_DIM, CONTROL_DIM, SCALAR>(arg),
	Q_(arg.Q_),
	R_(arg.R_),
	x_ref_(arg.x_ref_),
	u_ref_(arg.u_ref_)
{}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>* TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::clone () const {
		return new TermQuadratic(*this);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::setWeights(const state_matrix_double_t& Q, const control_matrix_double_t& R)
{
    Q_ = Q.template cast<SCALAR>();
    R_ = R.template cast<SCALAR>();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::setStateAndControlReference(
		const core::StateVector<STATE_DIM, SCALAR>& x_ref,
		const core::ControlVector<CONTROL_DIM, SCALAR>& u_ref)
{
	x_ref_ = x_ref.template cast<SCALAR>();
	u_ref_ = u_ref.template cast<SCALAR>();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SCALAR TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::evaluate(
		const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x,
		const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> &u,
		const SCALAR& t)
{
    Eigen::Matrix<SCALAR, STATE_DIM, 1> xDiff = (x-x_ref_.template cast<SCALAR>());
    Eigen::Matrix<SCALAR, CONTROL_DIM, 1> uDiff = (u-u_ref_.template cast<SCALAR>());

    return (xDiff.transpose() * Q_.template cast<SCALAR>() * xDiff + uDiff.transpose() * R_.template cast<SCALAR>() * uDiff)(0,0);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ct::core::StateVector<STATE_DIM, SCALAR> TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::stateDerivative(
		const ct::core::StateVector<STATE_DIM, SCALAR> &x,
		const ct::core::ControlVector<CONTROL_DIM, SCALAR> &u,
		const SCALAR& t)
{
    ct::core::StateVector<STATE_DIM, SCALAR> xDiff = (x-x_ref_);

    return xDiff.transpose() * Q_.transpose() + xDiff.transpose() * Q_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::state_matrix_t TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::stateSecondDerivative(
		const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t)
{
	return Q_ + Q_.transpose();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
core::ControlVector<CONTROL_DIM, SCALAR> TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::controlDerivative(
		const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t)
{
    core::ControlVector<CONTROL_DIM, SCALAR> uDiff = (u-u_ref_);

    return uDiff.transpose() * R_.transpose() + uDiff.transpose() * R_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::control_matrix_t TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::controlSecondDerivative(
		const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t)
{
	return R_ + R_.transpose() ;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::control_state_matrix_t TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::stateControlDerivative(
		const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t)
{
	return control_state_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::loadConfigFile(const std::string& filename, const std::string& termName, bool verbose)
{
		boost::property_tree::ptree pt;
		try{
		boost::property_tree::read_info(filename, pt);
		}catch(...){}
		this->name_ = pt.get<std::string>(termName +".name.", termName);

       loadMatrixCF(filename,"Q", Q_,termName);
       loadMatrixCF(filename,"R", R_,termName);
       loadMatrixCF(filename,"x_des", x_ref_,termName);
       loadMatrixCF(filename,"u_des", u_ref_,termName);
       if (verbose){
		   std::cout<<"Read Q as Q = \n"<<Q_<<std::endl;
		   std::cout<<"Read R as R = \n"<<R_<<std::endl;
		   std::cout<<"Read x_des as x_des = \n"<<x_ref_.transpose()<<std::endl;
		   std::cout<<"Read u_des as u_des = \n"<<u_ref_.transpose()<<std::endl;
       }
}
