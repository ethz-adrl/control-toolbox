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
#include "manifIntegration.h"

#include <ct/core/systems/SymplecticSystem.h>

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
template <typename SYM_MFD, size_t CONTROL_DIM, class Stepper>
class IntegratorSymplectic
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr size_t STATE_DIM = SYM_MFD::TangentDim;
    static constexpr size_t POS_DIM = SYM_MFD::PosDim;
    static constexpr size_t VEL_DIM = SYM_MFD::VelDim;

    static_assert(is_symplectic<SYM_MFD>::value, "Symplectic system: manifold must be defined as symplectic.");

    static_assert(STATE_DIM == (POS_DIM + VEL_DIM),
        "Symplectic system: state_dim must be the sum of position and velocity dim.");

    using SCALAR = typename SYM_MFD::Scalar;
    using pair_t = typename std::pair<Eigen::Matrix<SCALAR, POS_DIM, 1>, Eigen::Matrix<SCALAR, VEL_DIM, 1>>;

    using SymplecticSystem_t = SymplecticSystem<SYM_MFD, CONTROL_DIM>;
    using EventHandlerPtr = std::shared_ptr<EventHandler<SYM_MFD>>;
    using EventHandlerPtrVector = std::vector<EventHandlerPtr, Eigen::aligned_allocator<EventHandlerPtr>>;


    /**
	 * @brief      The constructor. This integrator can only treat symplectic
	 *             systems
	 *
	 * @param[in]  system  A core::system
	 */
    IntegratorSymplectic(const std::shared_ptr<SymplecticSystem_t> system,
        const EventHandlerPtrVector& eventHandlers = EventHandlerPtrVector(0));

    /**
	 * @brief      The constructor. This integrator can only treat symplectic
	 *             systems
	 *
	 * @param[in]  system  A core::system
	 */
    IntegratorSymplectic(const std::shared_ptr<SymplecticSystem_t> system, const EventHandlerPtr& eventHandler);


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
    void integrate_n_steps(SYM_MFD& state,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt,
        DiscreteArray<SYM_MFD>& stateTrajectory,
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
    void integrate_n_steps(SYM_MFD& state, const SCALAR& startTime, size_t numSteps, SCALAR dt);

    void reset();

private:
    /**
	 * @brief      Sets up the two system functions which act as a pair of
	 *             position and velocity update
	 */
    void setupSystem();
    SYM_MFD xCached_;  //! The cached state. This will be used for the system function

    //! the position system function
    std::function<void(const Eigen::Matrix<SCALAR, POS_DIM, 1>&, Eigen::Matrix<SCALAR, POS_DIM, 1>&)>
        systemFunctionPosition_;

    //! the velocity system function
    std::function<void(const Eigen::Matrix<SCALAR, VEL_DIM, 1>&, Eigen::Matrix<SCALAR, VEL_DIM, 1>&)>
        systemFunctionVelocity_;

    std::shared_ptr<SymplecticSystem_t> systemSymplectic_;

    Stepper stepper_;

    Observer<SYM_MFD> observer_;
};


/*******************************************************************
 * Defining the integrators
 *******************************************************************/

template <typename SYM_MFD, size_t CONTROL_DIM>
using IntegratorSymplecticEuler = IntegratorSymplectic<SYM_MFD,
    CONTROL_DIM,
    internal::symplectic_euler_t<SYM_MFD::PosDim, SYM_MFD::VelDim, typename SYM_MFD::Scalar>>;

template <typename SYM_MFD, size_t CONTROL_DIM>
using IntegratorSymplecticRk = IntegratorSymplectic<SYM_MFD,
    CONTROL_DIM,
    internal::symplectic_rk_t<SYM_MFD::PosDim, SYM_MFD::VelDim, typename SYM_MFD::Scalar>>;
}
}
