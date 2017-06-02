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

#ifndef CT_OPTCON_DMS_CORE_DERIVATIVES_BASE_H_
#define CT_OPTCON_DMS_CORE_DERIVATIVES_BASE_H_

#include <functional>

#include <ct/optcon/dms/dms_core/DmsDimensions.h>
#include <ct/optcon/dms/dms_core/OptVectorDms.h>

#include <ct/core/systems/System.h>

#include <ct/core/systems/linear/LinearSystem.h>

namespace ct {
namespace optcon {

/**
 * @ingroup    DMS
 *
 * @brief      Abstract base class to specify the sensitivity vectors to be
 *             integrated depending on the DMS settings
 *
 * @tparam     STATE_DIM       The state dimension
 * @tparam     CONTROL_DIM     The control dimension
 * @tparam     DERIVATIVE_DIM  The size of the derivative vector
 */
template<size_t STATE_DIM, size_t CONTROL_DIM, size_t DERIVATIVE_DIM>
class DerivativeBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const size_t STATE_D = STATE_DIM;
	static const size_t CONTROL_D = CONTROL_DIM;

	typedef DmsDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;

	typedef typename DIMENSIONS::state_vector_t  state_vector_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::time_array_t time_array_t;

	typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
	typedef typename DIMENSIONS::control_matrix_t control_matrix_t;
	typedef typename DIMENSIONS::state_control_matrix_t state_control_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::state_control_matrix_array_t state_control_matrix_array_t;

	typedef ct::core::StateVector<DERIVATIVE_DIM> derivative_vector_t;
	typedef ct::core::StateVectorArray<DERIVATIVE_DIM> derivative_traj_t;

	DerivativeBase() = delete;

	/**
	 * @brief      Default constructor
	 *
	 * @param[in]  controlledSystem  The nonlinear dynamics
	 * @param[in]  linearSystem      The linearized dynamics
	 * @param[in]  costFct           The cost function
	 * @param[in]  w                 The optimization variables
	 * @param[in]  controlSpliner    The control spliner
	 * @param[in]  timeGrid          The time grid
	 * @param[in]  shotNr            The shot number
	 * @param[in]  settings          The dms settings
	 */
	DerivativeBase(
			std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM>> controlledSystem,
			std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM>> linearSystem,
			std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>> costFct,
			std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w,
			std::shared_ptr<SplinerBase<control_vector_t>> controlSpliner,
			std::shared_ptr<TimeGrid> timeGrid,
			size_t shotNr,
			DmsSettings settings
	)
	:
		system_(controlledSystem),
		linearSystem_(linearSystem),
		costFct_(costFct),
		w_(w),
		controlSpliner_(controlSpliner),
		timeGrid_(timeGrid),
		shotNr_(shotNr),
		settings_(settings),
		A_(state_matrix_t::Zero()),
		B_(state_control_matrix_t::Zero()),
		state_(state_vector_t::Zero()),
		control_(control_vector_t::Zero()),
		dUdQi_(control_matrix_t::Zero()),
		dUdQip1_(control_matrix_t::Zero()),
		dUdHi_(control_vector_t::Zero()),
		dXdSi_(state_matrix_t::Zero()),
		dXdQi_(state_control_matrix_t::Zero()),
		dXdQip1_(state_control_matrix_t::Zero()),
		dXdHi_(state_vector_t::Zero()),
		costStateDer_(state_vector_t::Zero()),
		costControlDer_(control_vector_t::Zero())
	{}

	/**
	 * @brief      The destructor
	 */
	virtual ~DerivativeBase(){}

	/**
	 * @brief      Retrieves state and cost trajectories
	 *
	 * @param[out]      timeTraj   The time trajectory
	 * @param[out]      stateTraj  The state trajectory
	 * @param[out]      cost       The cost trajectory
	 */
	virtual void retrieveStateTrajectories(
			time_array_t& timeTraj,
			state_vector_array_t& stateTraj,
			double& cost
	) = 0;

	/**
	 * @brief      Retrieves the sensitivity trajectories
	 *
	 * @param[out] timeTraj          The time trajectory
	 * @param[out] stateTraj         The state trajectory
	 * @param[out] dXdSiTraj         The ODE sensitivity wrt s_i
	 * @param[out] dXdQiTraj         The ODE sensitivity wrt q_i
	 * @param[out] dXdQip1Traj       The ODE sensitivity wrt q_{i+1}
	 * @param[out] dXdHiTraj         The ODE sensitivity wrt h_i
	 * @param[out] costGradientSi    The cost gradient wrt s_i
	 * @param[out] costGradientQi    The cost gradient wrt q_i
	 * @param[out] costGradientQip1  The cost gradient wrt q_{i+1}
	 * @param[out] costGradientHi    The cost gradient wrt h_i
	 */
	virtual void retrieveTrajectories(
			time_array_t& timeTraj,
			state_vector_array_t& stateTraj,
			state_matrix_array_t& dXdSiTraj,
			state_control_matrix_array_t& dXdQiTraj,
			state_control_matrix_array_t& dXdQip1Traj,
			state_vector_array_t& dXdHiTraj,
			state_vector_t& costGradientSi,
			control_vector_t& costGradientQi,
			control_vector_t& costGradientQip1,
			double& costGradientHi
	) = 0;


	/**
	 * @brief      Initialize objects for the integration
	 */
	virtual void initForIntegration() = 0;

	/**
	 * @brief      Wraps up the integration, gets called after the integration
	 */
	virtual void wrapUpIntegration() = 0;

	/**
	 * @brief      Returns the initial derivative state
	 *
	 * @return     The initial derivative state
	 */
	virtual const derivative_vector_t getInitState() = 0;

	/**
	 * @brief      Returns the full derivative trajectory
	 *
	 * @return     The derivative trajectory
	 */
	virtual derivative_traj_t& stateTrajectory() = 0;

	/**
	 * @brief      Returns the time trajectory of the integration
	 *
	 * @return     The time trajectory of the integration
	 */
	virtual time_array_t& timeTrajectory() = 0;

	/**
	 * @brief      Returns the start time of the current shot
	 *
	 * @return     The shot start time
	 */
	double getShotStartTime() const {return timeGrid_->getShotStartTime(shotNr_);}

	/**
	 * @brief      Returns the end time of the current shot
	 *
	 * @return     The shot end time
	 */
	double getShotEndTime() const {return timeGrid_->getShotEndTime(shotNr_);}


protected:

	std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM> > system_;
	std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM>> linearSystem_;
	std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>> costFct_;
	std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w_;
	std::shared_ptr<SplinerBase<control_vector_t>> controlSpliner_;
	std::shared_ptr<TimeGrid> timeGrid_;

	size_t shotNr_;
	DmsSettings settings_;

	state_matrix_t A_;
	state_control_matrix_t B_;
	state_vector_t state_;
	control_vector_t control_;

	control_matrix_t dUdQi_;
	control_matrix_t dUdQip1_;
	control_vector_t dUdHi_;

	state_matrix_t dXdSi_;
	state_control_matrix_t dXdQi_;
	state_control_matrix_t dXdQip1_;
	state_vector_t dXdHi_;	

	state_vector_t costStateDer_;
	control_vector_t costControlDer_;

	/**
	 * @brief      Caches the updates A and B matrices
	 *
	 * @param[in]  t     The current time
	 */
	void updateMatrices(const ct::core::Time t)
	{
		A_ = linearSystem_->getDerivativeState(state_, control_, t);
		B_ = linearSystem_->getDerivativeControl(state_, control_, t);
	}

	/**
	 * @brief          Calculates the state derivative
	 *
	 * @param[in]      t           The current time
	 * @param[in, out] derivative  The total derivative vector
	 * @param[in]      count       The current index inside the derivative
	 *                             vector
	 *
	 * @tparam         Derived     The size of the derivative vector
	 *
	 * @return         The updated derivative vector index
	 */
	template<typename Derived>
	size_t stateDerivative(const ct::core::Time t, Eigen::MatrixBase<Derived>& derivative, size_t count)
	{
		const state_vector_t& xState(static_cast<const state_vector_t& >(state_));
		state_vector_t dxdtState;
		this->system_->computeDynamics(xState, t, dxdtState);
		derivative.segment(count, STATE_DIM) = dxdtState;
		count += STATE_DIM;
		return count;
	}

	/**
	 * @brief      Calculates the ODE sensivitiy wrt to s_i
	 *
	 * @param      derivative  The total derivative vector
	 * @param[in]  count       The current index inside the derivative vector
	 *
	 * @tparam     Derived     The size of the derivative vector
	 *
	 * @return     The updated derivative vector index
	 */
	template<typename Derived>
	size_t dxdsiDerivative(Eigen::MatrixBase<Derived>& derivative, size_t count)
	{
		state_matrix_t tmp = A_ * dXdSi_;
		derivative.segment(count, STATE_DIM*STATE_DIM) = Eigen::Map<Eigen::Matrix<double, STATE_DIM*STATE_DIM, 1>> (tmp.data(), STATE_DIM*STATE_DIM);
		count += STATE_DIM*STATE_DIM;
		return count;
	}

	/**
	 * @brief      Calculates the ODE sensivitiy wrt to q_i
	 *
	 * @param      derivative  The total derivative vector
	 * @param[in]  count       The current index inside the derivative vector
	 *
	 * @tparam     Derived     The size of the derivative vector
	 *
	 * @return     The updated derivative vector index
	 */
	template<typename Derived>
	size_t dxdqiDerivative(Eigen::MatrixBase<Derived>& derivative, size_t count)
	{
		state_control_matrix_t tmp = A_ * dXdQi_ + B_ * dUdQi_;
		derivative.segment(count, STATE_DIM*CONTROL_DIM) = Eigen::Map<Eigen::Matrix<double, STATE_DIM*CONTROL_DIM,1>> (tmp.data(), STATE_DIM*CONTROL_DIM);
		count += STATE_DIM*CONTROL_DIM;
		return count;
	}

	/**
	 * @brief      Calculates the ODE sensivitiy wrt to q_{i+1}
	 *
	 * @param      derivative  The total derivative vector
	 * @param[in]  count       The current index inside the derivative vector
	 *
	 * @tparam     Derived     The size of the derivative vector
	 *
	 * @return     The updated derivative vector index
	 */
	template<typename Derived>
	size_t dxdqip1Derivative(Eigen::MatrixBase<Derived>& derivative, size_t count)
	{
		state_control_matrix_t tmp = A_ * dXdQip1_ + B_ * dUdQip1_;
		derivative.segment(count, STATE_DIM*CONTROL_DIM) = Eigen::Map<Eigen::Matrix<double, STATE_DIM*CONTROL_DIM, 1>> (tmp.data(), STATE_DIM*CONTROL_DIM);
		count += STATE_DIM*CONTROL_DIM;
		return count;
	}

	/**
	 * @brief      Calculates the ODE sensivitiy wrt to h_i
	 *
	 * @param      derivative  The total derivative vector
	 * @param[in]  count       The current index inside the derivative vector
	 *
	 * @tparam     Derived     The size of the derivative vector
	 *
	 * @return     The updated derivative vector index
	 */
	template<typename Derived>
	size_t dxdhiDerivative(Eigen::MatrixBase<Derived>& derivative, size_t count)
	{
		state_vector_t tmp = A_ * dXdHi_ + B_ * dUdHi_;
		derivative.segment(count, STATE_DIM) = tmp;
		count += STATE_DIM;
		return count;
	}

	/**
	 * @brief      Calculates the cost gradient wrt time
	 *
	 * @param      derivative  The total derivative vector
	 * @param[in]  count       The current index inside the derivative vector
	 *
	 * @tparam     Derived     The size of the derivative vector
	 *
	 * @return     The updated derivative vector index
	 */
	template<typename Derived>
	size_t dLDerivative(Eigen::MatrixBase<Derived>& derivative, size_t count)
	{
		derivative(count) = costFct_->evaluateIntermediate();
		count += 1;
		return count;
	}

	/**
	 * @brief      Calculates the cost gradient wrt s_i
	 *
	 * @param      derivative  The total derivative vector
	 * @param[in]  count       The current index inside the derivative vector
	 *
	 * @tparam     Derived     The size of the derivative vector
	 *
	 * @return     The updated derivative vector index
	 */
	template<typename Derived>
	size_t dLdsiDerivative(Eigen::MatrixBase<Derived>& derivative, size_t count)
	{
		derivative.segment(count, STATE_DIM) = dXdSi_.transpose()*costStateDer_;
		count += STATE_DIM;
		return count;
	}

	/**
	 * @brief      Calculates the cost gradient wrt q_i
	 *
	 * @param      derivative  The total derivative vector
	 * @param[in]  count       The current index inside the derivative vector
	 *
	 * @tparam     Derived     The size of the derivative vector
	 *
	 * @return     The updated derivative vector index
	 */
	template<typename Derived>
	size_t dLdqiDerivative(Eigen::MatrixBase<Derived>& derivative, size_t count)
	{
		derivative.segment(count, CONTROL_DIM) = dXdQi_.transpose() * costStateDer_ + 
				dUdQi_.transpose() * costControlDer_;
		count += CONTROL_DIM;
		return count;
	}

	/**
	 * @brief      Calculates the cost gradient wrt q_{i+1}
	 *
	 * @param      derivative  The total derivative vector
	 * @param[in]  count       The current index inside the derivative vector
	 *
	 * @tparam     Derived     The size of the derivative vector
	 *
	 * @return     The updated derivative vector index
	 */
	template<typename Derived>
	size_t dLdqip1Derivative(Eigen::MatrixBase<Derived>& derivative, size_t count)
	{
		derivative.segment(count, CONTROL_DIM) = dXdQip1_.transpose() * costStateDer_ + 
				dUdQip1_.transpose() * costControlDer_;
		count += CONTROL_DIM;
		return count;
	}

	/**
	 * @brief      Calculates the cost gradient wrt h_i
	 *
	 * @param      derivative The total derivative vector
	 * @param[in]  count       The current index inside the derivative vector
	 *
	 * @tparam     Derived     The size of the derivative vector
	 *
	 * @return     The updated derivative vector index
	 */
	template<typename Derived>
	size_t dldHiDerivative(Eigen::MatrixBase<Derived>& derivative, size_t count)
	{
		derivative.segment(count, 1) = dUdHi_.transpose() * costControlDer_ 
				+ dXdHi_.transpose() * costStateDer_;
		count += 1;
		return count;
	}	

};

} // namespace optcon
} // namespace ct

#endif //DERIVATIVES_VECTORIZED_DERIVATIVES_BASE_HPP_
