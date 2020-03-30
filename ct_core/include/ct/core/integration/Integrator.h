/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <type_traits>
#include <functional>
#include <cmath>
#include <memory>

#include "EventHandler.h"
#include "Observer.h"
#include "eigenIntegration.h"
#include "manifIntegration.h"

#include "internal/StepperODEInt.h"
#include "internal/StepperODEIntDenseOutput.h"
#include "internal/StepperODEIntControlled.h"
#include "internal/StepperEulerCT.h"
#include "internal/StepperRK4CT.h"

#include <ct/core/types/AutoDiff.h>
#include <ct/core/types/arrays/DiscreteArray.h>
#include <ct/core/types/TypeTraits.h>

#include <ct/core/systems/System.h>

namespace ct {
namespace core {


/**
 * @brief      The available integration types
 */
enum IntegrationType
{
    EULER,
    RK4,
    MODIFIED_MIDPOINT,
    ODE45,
    RK5VARIABLE,
    RK78,
    BULIRSCHSTOER,
    EULERCT,
    RK4CT,
    EULER_SYM,
    RK_SYM
};


//! Standard Integrator
/*!
 * A standard Integrator for numerically solving Ordinary Differential Equations (ODEs) of the form
 *
 * \f[
 * \dot{x} = f(x,t)
 * \f]
 *
 * Unit test \ref IntegrationTest.cpp illustrates the use of Integrator.h
 *
 * @tparam STATE_DIM the size of the state vector
 * @tparam SCALAR The scalar type
 */
template <typename MANIFOLD>
class Integrator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SCALAR = typename MANIFOLD::Scalar;
    using TANGENT = typename MANIFOLD::Tangent;
    using System_t = System<MANIFOLD, CONTINUOUS_TIME>;
    using SystemPtr_t = std::shared_ptr<System_t>;

    using EventHandlerPtr = std::shared_ptr<EventHandler<MANIFOLD>>;
    using EventHandlerPtrVector = std::vector<EventHandlerPtr, Eigen::aligned_allocator<EventHandlerPtr>>;

    //! constructor
    /*!
	 * Creates a standard integrator
	 *
	 * @param system the system (ODE)
	 * @param eventHandlers optional event handler
	 * @param absErrTol optional absolute error tolerance (for variable step solvers)
	 * @param relErrTol optional relative error tolerance (for variable step solvers)
	 */
    Integrator(const SystemPtr_t& system,
        const IntegrationType& intType = IntegrationType::EULERCT,
        const EventHandlerPtrVector& eventHandlers = EventHandlerPtrVector(0));

    Integrator(const SystemPtr_t& system, const IntegrationType& intType, const EventHandlerPtr& eventHandler);

    /**
     * @brief Construct a new Integrator object
     */
    Integrator(const Integrator& other) = delete;

    /**
	 * @brief      Changes the integration type
	 *
	 * @param[in]  intType  The new integration type
	 */
    void changeIntegrationType(const IntegrationType& intType);

    /**
	 * @brief      Sets the adaptive error tolerances
	 *
	 * @param[in]  absErrTol  The absolute error tolerance
	 * @param[in]  relErrTol  The relative error tolerance
	 */
    void setApadativeErrorTolerances(const SCALAR absErrTol, const SCALAR& relErrTol);

    //! Equidistant integration based on number of time steps and step length
    /*!
	 * Integrates n steps forward from the current state recording the state and time trajectory.
	 * For a recording free version, see the function below.
	 *
	 * \warning Overrides the initial state
	 *
	 * @param state initial state, contains the final state after integration
	 * @param startTime start time of the integration
	 * @param numSteps number of steps to integrate forward
	 * @param dt step size (fixed also for variable step solvers)
	 * @param stateTrajectory state evolution over time
	 * @param timeTrajectory time trajectory corresponding to state trajectory
	 */
    void integrate_n_steps(MANIFOLD& state,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt,
        DiscreteArray<MANIFOLD>& stateTrajectory,
        tpl::TimeArray<SCALAR>& timeTrajectory);

    //! Equidistant integration based on number of time steps and step length
    /*!
	 *
	 * \warning Overrides the initial state
	 *
	 * @param state initial state, contains the final state after integration
	 * @param startTime start time of the integration
	 * @param numSteps number of steps to integrate forward
	 * @param dt step size (fixed also for variable step solvers)
	 */
    void integrate_n_steps(MANIFOLD& state, const SCALAR& startTime, size_t numSteps, SCALAR dt);

    //! Equidistant integration based on initial and final time as well as step length
    /*!
	 * Integrates forward from the current state recording the state and time trajectory.
	 * For a recording free version, see the function below.
	 *
	 * \warning Overrides the initial state
	 *
	 * @param state initial state, contains the final state after integration
	 * @param startTime start time of the integration
	 * @param finalTime the final time of the integration
	 * @param dt step size (fixed also for variable step solvers)
	 * @param stateTrajectory state evolution over time
	 * @param timeTrajectory time trajectory corresponding to state trajectory
	 */
    void integrate_const(MANIFOLD& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        SCALAR dt,
        DiscreteArray<MANIFOLD>& stateTrajectory,
        tpl::TimeArray<SCALAR>& timeTrajectory);

    //! Equidistant integration based on initial and final time as well as step length
    /*!
	 *
	 * \warning Overrides the initial state
	 *
	 * @param state initial state, contains the final state after integration
	 * @param startTime start time of the integration
	 * @param finalTime the final time of the integration
	 * @param dt step size (fixed also for variable step solvers)
	 */
    void integrate_const(MANIFOLD& state, const SCALAR& startTime, const SCALAR& finalTime, SCALAR dt);

    //! integrate forward from an initial to a final time using an adaptive scheme
    /*!
	 * Integrates forward in time from an initial to a final time. If an adaptive stepper is used,
	 * the time step is adjusted according to the tolerances set in the constructor. If a fixed step
	 * integrator is used, this function will do fixed step integration.
	 *
	 * Records state and time evolution. For a recording free version see function below
	 *
	 * \warning Overrides the initial state
	 *
	 * @param state initial state, contains the final state after integration
	 * @param startTime start time of the integration
	 * @param finalTime the final time of the integration
	 * @param stateTrajectory state evolution over time
	 * @param timeTrajectory time trajectory corresponding to state trajectory
	 * @param dtInitial step size (initial guess, for fixed step integrators it is fixed)
	 */
    void integrate_adaptive(MANIFOLD& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        DiscreteArray<MANIFOLD>& stateTrajectory,
        tpl::TimeArray<SCALAR>& timeTrajectory,
        const SCALAR dtInitial = SCALAR(0.01));

    //! integrate forward from an initial to a final time using an adaptive scheme
    /*!
	 * Integrates forward in time from an initial to a final time. If an adaptive stepper is used,
	 * the time step is adjusted according to the tolerances set in the constructor. If a fixed step
	 * integrator is used, this function will do fixed step integration.
	 *
	 * \warning Overrides the initial state
	 *
	 * @param state initial state, contains the final state after integration
	 * @param startTime start time of the integration
	 * @param finalTime the final time of the integration
	 * @param dtInitial step size (initial guess, for fixed step integrators it is fixed)
	 */
    void integrate_adaptive(MANIFOLD& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        SCALAR dtInitial = SCALAR(0.01));

    //! Integrate system using a given time trajectory
    /*!
	 * Integrates a system using a given time sequence
	 *
	 * \warning Overrides the initial state
	 *
	 * @param state initial state, contains the final state after integration
	 * @param timeTrajectory sequence of time stamps to perform the integration
	 * @param stateTrajectory the resulting state trajectory corresponding to the times provided
	 * @param dtInitial an initial guess for a time step (fixed for fixed step integrators)
	 */
    void integrate_times(MANIFOLD& state,
        const tpl::TimeArray<SCALAR>& timeTrajectory,
        DiscreteArray<MANIFOLD>& stateTrajectory,
        SCALAR dtInitial = SCALAR(0.01));

private:
    /**
	 * @brief      Initializes the custom ct steppers
	 *
	 * @param[in]  intType  The integration type
	 */
    void initializeCTSteppers(const IntegrationType& intType);
    /**
	 * @brief      Initializes the adaptive odeint steppers. The odeint steppers
	 *             only work for double currently
	 *
	 * @param[in]  intType  The integration type
	 *
	 */

    template <typename M = MANIFOLD, typename std::enable_if<ct::core::is_real_euclidean<M>::value, bool>::type = true>
    void initializeAdaptiveSteppers(const IntegrationType& intType);

    template <typename M = MANIFOLD,
        typename std::enable_if<!(ct::core::is_real_euclidean<M>::value), bool>::type = true>
    void initializeAdaptiveSteppers(const IntegrationType& intType);

#ifdef CPPADCG
    template <typename S = SCALAR>
    typename std::enable_if<std::is_same<S, ADCGScalar>::value, void>::type initializeODEIntSteppers(
        const IntegrationType& intType);
#endif

    /**
	 * @brief      Initializes the ODEint fixed size steppers for double type. Does not work for
	 *             ad types
	 *
	 * @param[in]  intType  The int type
	 *
	 */
    template <typename S = SCALAR>
    typename std::enable_if<std::is_same<S, double>::value, void>::type initializeODEIntSteppers(
        const IntegrationType& intType);

    //! resets the observer
    void reset();

    /**
	 * @brief      Retrieves the state and time trajectory from the observer.
	 *
	 * @param      stateTrajectory  The state trajectory
	 * @param      timeTrajectory   The time trajectory
	 */
    void retrieveTrajectoriesFromObserver(DiscreteArray<MANIFOLD>& stateTrajectory,
        tpl::TimeArray<SCALAR>& timeTrajectory);

    /**
	 * @brief      Retrieves the state trajectory from the observer
	 *
	 * @param      stateTrajectory  The state trajectory
	 */
    void retrieveStateVectorArrayFromObserver(DiscreteArray<MANIFOLD>& stateTrajectory);

    //! sets up the lambda function
    void setupSystem();

    SystemPtr_t system_;  //! pointer to the system

    //! the system function to integrate
    typename internal::StepperBase<MANIFOLD>::SystemFunction_t systemFunction_;

    std::shared_ptr<internal::StepperBase<MANIFOLD>> integratorStepper_;

    Observer<MANIFOLD> observer_;  //! observer
};


#ifdef CPPADCG
template <typename MANIFOLD>
template <typename S>
typename std::enable_if<std::is_same<S, ADCGScalar>::value, void>::type Integrator<MANIFOLD>::initializeODEIntSteppers(
    const IntegrationType& intType)
{
    // do nothing
}
#endif


template <typename MANIFOLD>
template <typename S>
typename std::enable_if<std::is_same<S, double>::value, void>::type Integrator<MANIFOLD>::initializeODEIntSteppers(
    const IntegrationType& intType)
{
    switch (intType)
    {
        case EULER:
        {
            integratorStepper_ = std::shared_ptr<internal::StepperODEInt<internal::euler_t<MANIFOLD>, MANIFOLD>>(
                new internal::StepperODEInt<internal::euler_t<MANIFOLD>, MANIFOLD>());
            break;
        }
        case RK4:
        {
            integratorStepper_ =
                std::shared_ptr<internal::StepperODEInt<internal::runge_kutta_4_t<MANIFOLD>, MANIFOLD>>(
                    new internal::StepperODEInt<internal::runge_kutta_4_t<MANIFOLD>, MANIFOLD>());
            break;
        }
        case MODIFIED_MIDPOINT:
        {
            if (is_euclidean<MANIFOLD>::value)
            {
                integratorStepper_ =
                    std::shared_ptr<internal::StepperODEInt<internal::modified_midpoint_t<MANIFOLD>, MANIFOLD>>(
                        new internal::StepperODEInt<internal::modified_midpoint_t<MANIFOLD>, MANIFOLD>());
            }
            else
                throw std::runtime_error(
                    "MODIFIED_MIDPOINT stepping not supported on manifolds. Use a different stepper.");
            break;
        }

        case RK78:
        {
            integratorStepper_ =
                std::shared_ptr<internal::StepperODEInt<internal::runge_kutta_fehlberg78_t<MANIFOLD>, MANIFOLD>>(
                    new internal::StepperODEInt<internal::runge_kutta_fehlberg78_t<MANIFOLD>, MANIFOLD>());

            break;
        }
        default:
            break;
    }
}


template <typename MANIFOLD>
template <typename M, typename std::enable_if<!(ct::core::is_real_euclidean<M>::value), bool>::type>
void Integrator<MANIFOLD>::initializeAdaptiveSteppers(const IntegrationType& intType)
{
    switch (intType)
    {
        case ODE45:
        {
            throw std::runtime_error(
                "Integrator ODE45 on manifolds currently not supported yet. Implementation of error measures in "
                "manifIntegration.h required.");
            break;
        }
        case RK5VARIABLE:
        {
            throw std::runtime_error(
                "Integrator RK5VARIABLE on manifolds currently not supported yet. Implementation of error measures in "
                "manifIntegration.h required.");
            break;
        }
        case BULIRSCHSTOER:
        {
            throw std::runtime_error(
                "Integrator BULIRSCHSTOER on manifolds currently not supported yet. Implementation of error measures "
                "in manifIntegration.h required.");
            break;
        }
        default:
            break;
    }
}

template <typename MANIFOLD>
template <typename M, typename std::enable_if<ct::core::is_real_euclidean<M>::value, bool>::type>
void Integrator<MANIFOLD>::initializeAdaptiveSteppers(const IntegrationType& intType)
{
    switch (intType)
    {
        case ODE45:
        {
            integratorStepper_ =
                std::shared_ptr<internal::StepperODEIntControlled<internal::runge_kutta_dopri5_t<MANIFOLD>, MANIFOLD>>(
                    new internal::StepperODEIntControlled<internal::runge_kutta_dopri5_t<MANIFOLD>, MANIFOLD>());
            break;
        }

        case RK5VARIABLE:
        {
            integratorStepper_ =
                std::shared_ptr<internal::StepperODEIntDenseOutput<internal::runge_kutta_dopri5_t<MANIFOLD>, MANIFOLD>>(
                    new internal::StepperODEIntDenseOutput<internal::runge_kutta_dopri5_t<MANIFOLD>, MANIFOLD>());
            break;
        }

        case BULIRSCHSTOER:
        {
            integratorStepper_ =
                std::shared_ptr<internal::StepperODEInt<internal::bulirsch_stoer_t<MANIFOLD>, MANIFOLD>>(
                    new internal::StepperODEInt<internal::bulirsch_stoer_t<MANIFOLD>, MANIFOLD>());
            break;
        }
        default:
            break;
    }
}

template <size_t DIM, typename SCALAR = double>
using EuclideanIntegrator = Integrator<EuclideanState<DIM, SCALAR>>;

}  // namespace core
}  // namespace ct
