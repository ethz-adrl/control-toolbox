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

#ifndef CT_OPTCON_COSTFUNCTION_COSTFUNCTIONQUADRATICTRACKING_HPP_
#define CT_OPTCON_COSTFUNCTION_COSTFUNCTIONQUADRATICTRACKING_HPP_

#include <ct/core/core.h>

namespace ct{
namespace optcon{

//! A cost function dedicated to tracking
/*!
 * An example for using this cost function is given in unit test \ref TrackingTest.cpp
 * \warning this is a legacy class which will be removed in a future release.
 * \todo remove.
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class CostFunctionQuadraticTracking : public optcon::CostFunctionQuadratic< STATE_DIM, CONTROL_DIM, SCALAR>
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
	typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;

	typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t;
	typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
	typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> control_feedback_t;


	typedef typename core::StateVectorArray<STATE_DIM, SCALAR> state_vector_array_t;
	typedef typename core::ControlVectorArray<CONTROL_DIM, SCALAR> control_vector_array_t;
	typedef typename core::tpl::TimeArray<SCALAR> time_array_t;


	CostFunctionQuadraticTracking(
		const state_matrix_t& Q,
		const control_matrix_t& R,
		const state_matrix_t& Q_final, 
		const core::InterpolationType& stateSplineType,
		const core::InterpolationType& controlSplineType,
		const bool trackControlTrajectory = false):
			x_deviation_(state_vector_t::Zero()),
			u_deviation_(control_vector_t::Zero()),
			Q_(Q),
			R_(R),
			Q_final_(Q_final),
			x_traj_ref_(stateSplineType),
			u_traj_ref_(controlSplineType),
			trackControlTrajectory_(trackControlTrajectory)
	{}

	CostFunctionQuadraticTracking(const CostFunctionQuadraticTracking& arg) :
		optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>(arg),
		x_deviation_(arg.x_deviation_),
		Q_(arg.Q_),
		u_deviation_(arg.u_deviation_),
		R_(arg.R_),
		Q_final_(arg.Q_final_),
		x_traj_ref_(arg.x_traj_ref_),
		u_traj_ref_(arg.u_traj_ref_),
		trackControlTrajectory_(arg.trackControlTrajectory_)
	{}

	CostFunctionQuadraticTracking<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override {
		return new CostFunctionQuadraticTracking<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
	}

	virtual ~CostFunctionQuadraticTracking() {}

	virtual void setCurrentStateAndControl(const state_vector_t& x, const control_vector_t& u, const SCALAR& t) override{
		this->x_ = x;
		this->u_ = u;

		x_deviation_ = x - x_traj_ref_.eval(t);

		if(trackControlTrajectory_)
			u_deviation_ = u - u_traj_ref_.eval(t);
		else
			u_deviation_ = u ;
	}

	void updateTrajectories(
		const core::StateTrajectory<STATE_DIM, SCALAR>& xTraj,
		const core::ControlTrajectory<CONTROL_DIM, SCALAR>& uTraj)
	{
		x_traj_ref_ = xTraj;
		u_traj_ref_ = uTraj;
	}

	core::StateTrajectory<STATE_DIM, SCALAR>& getReferenceStateTrajectory() {return x_traj_ref_;}

	core::ControlTrajectory<CONTROL_DIM, SCALAR>& getReferenceControlTrajectory() {return u_traj_ref_;}


	SCALAR evaluateIntermediate() override
	{
	  SCALAR costQ = x_deviation_.transpose() * Q_ * x_deviation_;
	  SCALAR costR = u_deviation_.transpose() * R_ * u_deviation_;
	  return costQ + costR;
	}

	state_vector_t stateDerivativeIntermediate() override
	{
		return  2*Q_ * x_deviation_;
	}

	state_matrix_t stateSecondDerivativeIntermediate() override
	{
		return 2*Q_;
	}

	control_vector_t controlDerivativeIntermediate() override
	{
		return 2*R_ * u_deviation_;
	}

	control_matrix_t controlSecondDerivativeIntermediate() override
	{
		return 2*R_;
	}


	control_feedback_t stateControlDerivativeIntermediate() override
	{
		return control_feedback_t::Zero();
	}


	SCALAR evaluateTerminal() override
	{
		return  x_deviation_.transpose() * Q_final_ * x_deviation_;
	}

	state_vector_t stateDerivativeTerminal() override
	{
		return 2*Q_final_ * x_deviation_;
	}

	state_matrix_t stateSecondDerivativeTerminal() override
	{
		return 2*Q_final_;
	}


protected:

	state_vector_t x_deviation_;
	state_matrix_t Q_;
	
	control_vector_t u_deviation_;
	control_matrix_t R_;
	state_matrix_t Q_final_;

	// the reference trajectories to be tracked
	ct::core::StateTrajectory<STATE_DIM, SCALAR> x_traj_ref_;
	ct::core::ControlTrajectory<CONTROL_DIM, SCALAR> u_traj_ref_;

	// Option whether the control trajectory deviation shall be penalized or not
	bool trackControlTrajectory_;

};


}	// namespace optcon
}	// namespace ct


#endif //CT_OPTCON_COSTFUNCTION_COSTFUNCTIONQUADRATICTRACKING_HPP_
