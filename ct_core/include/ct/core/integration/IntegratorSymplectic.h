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


#ifndef CT_CORE_INTEGRATOR_SYMPLECTIC_H_
#define CT_CORE_INTEGRATOR_SYMPLECTIC_H_

#include <type_traits>
#include <functional>
#include <cmath>

#include <boost/numeric/odeint.hpp>

#include "eigenIntegration.h"

#include <ct/core/systems/SymplecticSystem.h>

#include "internal/steppers.h"


namespace ct {
namespace core {

/**
 * @brief      This class wraps the symplectic integrators from boost to this
 *             toolbox.
 *
 * @tparam     POS_DIM      The position dimension
 * @tparam     VEL_DIM      The velocity dimension
 * @tparam     CONTROL_DIM  The control dimension
 * @tparam     Stepper      The stepper type
 */
template <size_t POS_DIM, size_t VEL_DIM, size_t CONTROL_DIM, class Stepper, typename SCALAR = double>
class IntegratorSymplectic
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef typename std::pair<Eigen::Matrix<SCALAR, POS_DIM, 1>, Eigen::Matrix<SCALAR, VEL_DIM, 1>> pair_t;


	/**
	 * @brief      The constructor. This integrator can only treat symplectic
	 *             systems
	 *
	 * @param[in]  system  A core::system
	 */
	IntegratorSymplectic(const std::shared_ptr<SymplecticSystem<POS_DIM, VEL_DIM, CONTROL_DIM, SCALAR> > system) :
	systemSymplectic_(system)
	{
		setupSystem();
	}


	/**
	 * @brief        Equidistant integration based on number of time steps and
	 *               step length
	 *
	 * @param[inout] state            The initial state, gets updated during
	 *                                integration
	 * @param[in]    startTime        The starting time of the integration
	 * @param[in]    numSteps         The number of integration steps
	 * @param[in]    dt               The integration time step
	 * @param[out]   stateTrajectory  The resulting state trajectory
	 * @param[out]   timeTrajectory   The resulting time trajectory
	 */
	void integrate_n_steps(
			StateVector<POS_DIM + VEL_DIM, SCALAR>& state,
			const SCALAR& startTime,
			size_t numSteps,
			SCALAR dt,
			StateVectorArray<POS_DIM + VEL_DIM, SCALAR>& stateTrajectory,
			tpl::TimeArray<SCALAR>& timeTrajectory
	) {
		pair_t xPair;
		xPair.first = state.head(POS_DIM);
		xPair.second = state.tail(VEL_DIM);
		SCALAR time = startTime;
		stateTrajectory.push_back(state);
		timeTrajectory.push_back(time);

		for(size_t i = 0; i < numSteps; ++i)
		{
			xCached_ = state;
			stepper_.do_step(std::make_pair(systemFunctionPosition_, systemFunctionVelocity_), xPair, time, dt);
			state.head(POS_DIM) = xPair.first();
			state.tail(VEL_DIM) = xPair.second();
			time += dt;
			stateTrajectory.push_back(state);
			timeTrajectory.push_back(time);
		}
	}

	/**
	 * @brief        Equidistant integration based on number of time steps and
	 *               step length
	 *
	 * @param[inout] state      The initial state, gets updated during
	 *                          integration
	 * @param[in]    startTime  The starting time of the integration
	 * @param[in]    numSteps   The number of integration steps
	 * @param[in]    dt         The integration time step
	 */
	void integrate_n_steps(
			StateVector<POS_DIM + VEL_DIM, SCALAR>& state,
			const SCALAR& startTime,
			size_t numSteps,
			SCALAR dt
	) {
		pair_t xPair;
		xPair.first = state.head(POS_DIM);
		xPair.second = state.tail(VEL_DIM);
		SCALAR time = startTime;
		for(size_t i = 0; i < numSteps; ++i)
		{

			xCached_ = state;
			stepper_.do_step(std::make_pair(systemFunctionPosition_, systemFunctionVelocity_), xPair, time, dt);
			state.head(POS_DIM) = xPair.first;
			state.tail(VEL_DIM) = xPair.second;
			time += dt;
		}

	}

private:

	/**
	 * @brief      Sets up the two system functions which act as a pair of
	 *             position and velocity update
	 */
	void setupSystem() {

		systemFunctionPosition_ = [this](const Eigen::Matrix<SCALAR, VEL_DIM, 1>& v, Eigen::Matrix<SCALAR, POS_DIM, 1>& dxdt) {
			const StateVector<POS_DIM, SCALAR>& vState(static_cast<const StateVector<POS_DIM, SCALAR>& >(v));
			StateVector<POS_DIM, SCALAR>& dxdtState(static_cast<StateVector<POS_DIM, SCALAR>& >(dxdt));
			systemSymplectic_->computePdot(xCached_, vState, dxdtState);
		};

		systemFunctionVelocity_ = [this](const Eigen::Matrix<SCALAR, POS_DIM, 1>& x, Eigen::Matrix<SCALAR, VEL_DIM, 1>& dvdt) {
			const StateVector<VEL_DIM, SCALAR>& xState(static_cast<const StateVector<VEL_DIM, SCALAR>& >(x));
			StateVector<VEL_DIM, SCALAR>& dvdtState(static_cast<StateVector<VEL_DIM, SCALAR>& >(dvdt));
			systemSymplectic_->computeVdot(xCached_, xState, dvdtState);
		};
	}

	StateVector<POS_DIM + VEL_DIM, SCALAR> xCached_; //! The cached state. This will be used for the system function
	                                         //! calls

	std::function<void (const Eigen::Matrix<SCALAR, POS_DIM, 1>&, Eigen::Matrix<SCALAR, POS_DIM, 1>&)> systemFunctionPosition_; //! the position system function
	std::function<void (const Eigen::Matrix<SCALAR, VEL_DIM, 1>&, Eigen::Matrix<SCALAR, VEL_DIM, 1>&)> systemFunctionVelocity_; //! the velocity system function

	std::shared_ptr<SymplecticSystem<POS_DIM, VEL_DIM, CONTROL_DIM, SCALAR>> systemSymplectic_;

	Stepper stepper_;
};



/*******************************************************************
 * Defining the integrators
 *******************************************************************/

template <size_t POS_DIM, size_t VEL_DIM, size_t CONTROL_DIM, typename SCALAR = double>
using IntegratorSymplecticEuler = IntegratorSymplectic<POS_DIM, VEL_DIM, CONTROL_DIM, internal::symplectic_euler_t<POS_DIM, VEL_DIM, SCALAR>, SCALAR>;

template <size_t POS_DIM, size_t VEL_DIM, size_t CONTROL_DIM, typename SCALAR = double>
using IntegratorSymplecticRk = IntegratorSymplectic<POS_DIM, VEL_DIM, CONTROL_DIM, internal::symplectic_rk_t<POS_DIM, VEL_DIM, SCALAR>, SCALAR>;

}
}

#endif /* CT_CORE_INTEGRATOR_SYMPLECTIC_H_ */
