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

#ifndef CT_COSTFUNCTION_TERMQUADTRACKING_HPP_
#define CT_COSTFUNCTION_TERMQUADTRACKING_HPP_

#include "TermBase.hpp"

#include <ct/core/core.h>

namespace ct {
namespace optcon {

/**
 * \ingroup CostFunction
 *
 * \brief A quadratic tracking term of type \f$ J(t) = (x_{ref}(t) - x)^T Q (x_{ref}(t) - x) + (u_{ref}(t) - u)^T R (u_{ref}(t) - u) \f$
 *
 * An example for using this term is given in unit test \ref TrackingTest.cpp
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename S = double, typename TIME_SCALAR = S>
class TermQuadTracking : public TermBase<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR> {

public:
	typedef S SCALAR;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	CT_OPTCON_DEFINE_TERM_TYPES

	TermQuadTracking();

	TermQuadTracking(
			const state_matrix_t& Q,
			const control_matrix_t& R,
			const core::InterpolationType& stateSplineType,
			const core::InterpolationType& controlSplineType,
			const bool trackControlTrajectory = false);

	TermQuadTracking(const TermQuadTracking& arg);

	virtual ~TermQuadTracking(){}
	
	TermQuadTracking<STATE_DIM, CONTROL_DIM, S, TIME_SCALAR>* clone () const override;

	void setWeights(const state_matrix_double_t& Q, const control_matrix_double_t& R);

	void setStateAndControlReference(
			const core::StateTrajectory<STATE_DIM>& xTraj,
			const core::ControlTrajectory<CONTROL_DIM>& uTraj);

	S evaluate(const Eigen::Matrix<S, STATE_DIM, 1> &x, const Eigen::Matrix<S, CONTROL_DIM, 1> &u, const S& t) override;
	
	core::StateVector<STATE_DIM, S> stateDerivative(const core::StateVector<STATE_DIM, S> &x,
			const core::ControlVector<CONTROL_DIM, S> &u, const S& t) override;

	state_matrix_t stateSecondDerivative(const core::StateVector<STATE_DIM, S> &x,
			const core::ControlVector<CONTROL_DIM, S> &u, const S& t) override;
	
	core::ControlVector<CONTROL_DIM, S> controlDerivative(const core::StateVector<STATE_DIM, S> &x,
			const core::ControlVector<CONTROL_DIM, S> &u, const S& t) override;
	
	control_matrix_t controlSecondDerivative(const core::StateVector<STATE_DIM, S> &x,
			const core::ControlVector<CONTROL_DIM, S> &u, const S& t) override;

	control_state_matrix_t stateControlDerivative(const core::StateVector<STATE_DIM, S> &x,
			const core::ControlVector<CONTROL_DIM, S> &u, const S& t) override;
	
	virtual void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false) override;

protected:

	state_matrix_t Q_;
	control_matrix_t R_;

	// the reference trajectories to be tracked
	ct::core::StateTrajectory<STATE_DIM> x_traj_ref_;
	ct::core::ControlTrajectory<CONTROL_DIM> u_traj_ref_;

	// Option whether the control trajectory deviation shall be penalized or not
	bool trackControlTrajectory_;

};

#include "implementation/TermQuadTracking.hpp"

} // namespace optcon
} // namespace ct

#endif
