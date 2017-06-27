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


template <size_t STATE_DIM, size_t CONTROL_DIM, typename S, typename TIME_SCALAR>
TermQuadTracking<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>::TermQuadTracking(
		const state_matrix_t& Q,
		const control_matrix_t& R,
		const core::InterpolationType& stateSplineType,
		const core::InterpolationType& controlSplineType,
		const bool trackControlTrajectory) :
		Q_(Q),
		R_(R),
		x_traj_ref_(stateSplineType),
		u_traj_ref_(controlSplineType),
		trackControlTrajectory_(trackControlTrajectory)
{}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename S, typename TIME_SCALAR>
TermQuadTracking<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>::TermQuadTracking() {
	Q_.setIdentity();	// default values
	R_.setIdentity();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename S, typename TIME_SCALAR>
TermQuadTracking<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>::TermQuadTracking(const TermQuadTracking<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>& arg):
	TermBase<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>(arg),
	Q_(arg.Q_),
	R_(arg.R_),
	x_traj_ref_(arg.x_traj_ref_),
	u_traj_ref_(arg.u_traj_ref_),
	trackControlTrajectory_(arg.trackControlTrajectory_)
{}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename S, typename TIME_SCALAR>
TermQuadTracking<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>* TermQuadTracking<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>::clone () const {
		return new TermQuadTracking(*this);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename S, typename TIME_SCALAR>
void TermQuadTracking<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>::setWeights(const state_matrix_double_t& Q, const control_matrix_double_t& R)
{
    Q_ = Q.template cast<S>();
    R_ = R.template cast<S>();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename S, typename TIME_SCALAR>
void TermQuadTracking<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>::setStateAndControlReference(
		const core::StateTrajectory<STATE_DIM>& xTraj,
		const core::ControlTrajectory<CONTROL_DIM>& uTraj)
{
	x_traj_ref_ = xTraj;
	u_traj_ref_ = uTraj;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename S, typename TIME_SCALAR>
S TermQuadTracking<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>::evaluate(
		const Eigen::Matrix<S, STATE_DIM, 1> &x,
		const Eigen::Matrix<S, CONTROL_DIM, 1> &u,
		const S& t)
{
    Eigen::Matrix<S, STATE_DIM, 1> xDiff = x-x_traj_ref_.eval(t);

    Eigen::Matrix<S, CONTROL_DIM, 1> uDiff;

    if(trackControlTrajectory_)
    	uDiff = u-u_traj_ref_.eval(t);
    else
    	uDiff = u;

    return (xDiff.transpose() * Q_.template cast<S>() * xDiff + uDiff.transpose() * R_.template cast<S>() * uDiff)(0,0);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename S, typename TIME_SCALAR>
ct::core::StateVector<STATE_DIM, S> TermQuadTracking<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>::stateDerivative(
		const ct::core::StateVector<STATE_DIM, S> &x,
		const ct::core::ControlVector<CONTROL_DIM, S> &u,
		const S& t)
{
    Eigen::Matrix<S, STATE_DIM, 1> xDiff = x-x_traj_ref_.eval(t);

    return xDiff.transpose() * Q_.transpose() + xDiff.transpose() * Q_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename S, typename TIME_SCALAR>
typename TermQuadTracking<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>::state_matrix_t TermQuadTracking<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>::stateSecondDerivative(
		const core::StateVector<STATE_DIM, S> &x, const core::ControlVector<CONTROL_DIM, S> &u, const S& t)
{
	return Q_ + Q_.transpose();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename S, typename TIME_SCALAR>
core::ControlVector<CONTROL_DIM, S> TermQuadTracking<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>::controlDerivative(
		const core::StateVector<STATE_DIM, S> &x, const core::ControlVector<CONTROL_DIM, S> &u, const S& t)
{
    Eigen::Matrix<S, CONTROL_DIM, 1> uDiff;

    if(trackControlTrajectory_)
    	uDiff = u-u_traj_ref_.eval(t);
    else
    	uDiff = u;

    return uDiff.transpose() * R_.transpose() + uDiff.transpose() * R_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename S, typename TIME_SCALAR>
typename TermQuadTracking<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>::control_matrix_t TermQuadTracking<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>::controlSecondDerivative(
		const core::StateVector<STATE_DIM, S> &x, const core::ControlVector<CONTROL_DIM, S> &u, const S& t)
{
	return R_ + R_.transpose() ;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename S, typename TIME_SCALAR>
typename TermQuadTracking<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>::control_state_matrix_t TermQuadTracking<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>::stateControlDerivative(
		const core::StateVector<STATE_DIM, S> &x, const core::ControlVector<CONTROL_DIM, S> &u, const S& t)
{
	return control_state_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename S, typename TIME_SCALAR>
void TermQuadTracking<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>::loadConfigFile(const std::string& filename, const std::string& termName, bool verbose)
{
       loadMatrixCF(filename,"Q", Q_,termName);
       loadMatrixCF(filename,"R", R_,termName);
       if (verbose){
		   std::cout<<"Read Q as Q = \n"<<Q_<<std::endl;
		   std::cout<<"Read R as R = \n"<<R_<<std::endl;
       }
}
