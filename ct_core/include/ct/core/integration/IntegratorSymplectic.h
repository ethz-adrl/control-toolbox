/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <type_traits>
#include <functional>
#include <cmath>

#include <boost/numeric/odeint.hpp>

#include "eigenIntegration.h"

#include <ct/core/systems/continuous_time/SymplecticSystem.h>

#include "internal/SteppersODEIntDefinitions.h"


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

    typedef std::shared_ptr<EventHandler<POS_DIM + VEL_DIM, SCALAR>> EventHandlerPtr;
    typedef std::vector<EventHandlerPtr, Eigen::aligned_allocator<EventHandlerPtr>> EventHandlerPtrVector;


    /**
	 * @brief      The constructor. This integrator can only treat symplectic
	 *             systems
	 *
	 * @param[in]  system  A core::system
	 */
    IntegratorSymplectic(const std::shared_ptr<SymplecticSystem<POS_DIM, VEL_DIM, CONTROL_DIM, SCALAR>> system,
        const EventHandlerPtrVector& eventHandlers = EventHandlerPtrVector(0));

    /**
	 * @brief      The constructor. This integrator can only treat symplectic
	 *             systems
	 *
	 * @param[in]  system  A core::system
	 */
    IntegratorSymplectic(const std::shared_ptr<SymplecticSystem<POS_DIM, VEL_DIM, CONTROL_DIM, SCALAR>> system,
        const EventHandlerPtr& eventHandler);


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
    void integrate_n_steps(StateVector<POS_DIM + VEL_DIM, SCALAR>& state,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt,
        StateVectorArray<POS_DIM + VEL_DIM, SCALAR>& stateTrajectory,
        tpl::TimeArray<SCALAR>& timeTrajectory);

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
    void integrate_n_steps(StateVector<POS_DIM + VEL_DIM, SCALAR>& state,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt);

    void reset();

private:
    /**
	 * @brief      Sets up the two system functions which act as a pair of
	 *             position and velocity update
	 */
    void setupSystem();
    StateVector<POS_DIM + VEL_DIM, SCALAR> xCached_;  //! The cached state. This will be used for the system function

    std::function<void(const Eigen::Matrix<SCALAR, POS_DIM, 1>&, Eigen::Matrix<SCALAR, POS_DIM, 1>&)>
        systemFunctionPosition_;  //! the position system function
    std::function<void(const Eigen::Matrix<SCALAR, VEL_DIM, 1>&, Eigen::Matrix<SCALAR, VEL_DIM, 1>&)>
        systemFunctionVelocity_;  //! the velocity system function

    std::shared_ptr<SymplecticSystem<POS_DIM, VEL_DIM, CONTROL_DIM, SCALAR>> systemSymplectic_;

    Stepper stepper_;

    Observer<POS_DIM + VEL_DIM, SCALAR> observer_;  //! observer
};


/*******************************************************************
 * Defining the integrators
 *******************************************************************/

template <size_t POS_DIM, size_t VEL_DIM, size_t CONTROL_DIM, typename SCALAR = double>
using IntegratorSymplecticEuler =
    IntegratorSymplectic<POS_DIM, VEL_DIM, CONTROL_DIM, internal::symplectic_euler_t<POS_DIM, VEL_DIM, SCALAR>, SCALAR>;

template <size_t POS_DIM, size_t VEL_DIM, size_t CONTROL_DIM, typename SCALAR = double>
using IntegratorSymplecticRk =
    IntegratorSymplectic<POS_DIM, VEL_DIM, CONTROL_DIM, internal::symplectic_rk_t<POS_DIM, VEL_DIM, SCALAR>, SCALAR>;
}
}
