/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {
namespace internal {

/**
 * @brief      This class serves as a common interface between the ODEInt and
 *             our custom integrators
 *
 * @tparam     MATRIX  The matrix type to be integrated
 * @tparam     SCALAR      The scalar type
 */
template <typename MATRIX, typename SCALAR = double>
class StepperBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StepperBase() : absErrTol_(SCALAR(1e-8)), relErrTol_(SCALAR(1e-8)) {}
    virtual ~StepperBase() {}
    /**
     * @brief         Performs numSteps integration steps
     *
     * @param[in]     rhs        The ODE to be integrated
     * @param[in,out] state      The state
     * @param[in]     startTime  The start time
     * @param[in]     numSteps   The number of integration steps
     * @param[in]     dt         The integration timestep
     */
    virtual void integrate_n_steps(const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt)
    {
        throw std::runtime_error("Integrate_n_steps not implemented for the stepper type");
    }

    /**
     * @brief         Performs numSteps integration steps
     *
     * @param[in]     observer   The observer
     * @param[in]     rhs        The ODE to be integrated
     * @param[in,out] state      The state
     * @param[in]     startTime  The start time
     * @param[in]     numSteps   The number steps
     * @param[in]     dt         The integration timestep
     */
    virtual void integrate_n_steps(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
        const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt)
    {
        throw std::runtime_error("Integrate_n_steps not implemented for the stepper type");
    }


    /**
     * @brief         Equidistant integration based on initial and final time as well as step length
     *
     * @param[in]     rhs        The ODE to be integrated
     * @param[in,out] state      The state
     * @param[in]     startTime  The start time
     * @param[in]     finalTime  The final time
     * @param[in]     dt         The integration timestep
     */
    virtual void integrate_const(const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        SCALAR dt)
    {
        throw std::runtime_error("integrate_const not implemented for the stepper type");
    }

    /**
     * @brief         Equidistant integration based on initial and final time as well as step length
     *
     * @param[in]     observer   The observer
     * @param[in]     rhs        The ODE to be integrated
     * @param[in,out] state      The state
     * @param[in]     startTime  The start time
     * @param[in]     finalTime  The final time
     * @param[in]     dt         The integration timestep
     */
    virtual void integrate_const(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
        const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        SCALAR dt)
    {
        throw std::runtime_error("integrate_const not implemented for the stepper type");
    }

    /**
     * @brief         Integrates forward in time from an initial to a final
     *                time. If an adaptive stepper is used, the time step is
     *                adjusted according to the error tolerances. If a fixed
     *                step integrator is used, this function will do fixed step
     *                integration.
     *
     * @param[in]     rhs        The ODE to be integrated
     * @param[in,out] state      The state
     * @param[in]     startTime  The start time
     * @param[in]     finalTime  The final time
     * @param[in]     dtInitial  The initial integration timestep
     */
    virtual void integrate_adaptive(const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        SCALAR dtInitial = SCALAR(0.01))
    {
        throw std::runtime_error("integrate_adaptive not implemented for the stepper type");
    }

    /**
     * @brief         Integrates forward in time from an initial to a final
     *                time. If an adaptive stepper is used, the time step is
     *                adjusted according to the error tolerances. If a fixed
     *                step integrator is used, this function will do fixed step
     *                integration.
     *
     * @param[in]     observer   The observer
     * @param[in]     rhs        The ODE to be integrated
     * @param[in,out] state      The state
     * @param[in]     startTime  The start time
     * @param[in]     finalTime  The final time
     * @param[in]     dtInitial  The initial integration timestep
     */
    virtual void integrate_adaptive(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
        const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        const SCALAR dtInitial = SCALAR(0.01))
    {
        throw std::runtime_error("integrate_adaptive not implemented for the stepper type");
    }

    /**
     * @brief         Integrates a system using a given time sequence
     *
     * @param[in]     observer        The observer
     * @param[in]     rhs             The ODE to be integrated
     * @param[in,out] state           The state
     * @param[in]     timeTrajectory  The time trajectory
     * @param[in]     dtInitial       The initial integration timestep
     */
    virtual void integrate_times(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
        const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const tpl::TimeArray<SCALAR>& timeTrajectory,
        SCALAR dtInitial = SCALAR(0.01))
    {
        throw std::runtime_error("integrate_times not implemented for the stepper type");
    }

    /**
     * @brief      Sets the adaptive error tolerances.
     *
     * @param[in]  absErrTol  The absolute error tolerance
     * @param[in]  relErrTol  The relative error tolerance
     */
    void setAdaptiveErrorTolerances(const SCALAR absErrTol, const SCALAR& relErrTol)
    {
        absErrTol_ = absErrTol;
        relErrTol_ = relErrTol;
    }

protected:
    SCALAR absErrTol_;
    SCALAR relErrTol_;
};
}
}
}
