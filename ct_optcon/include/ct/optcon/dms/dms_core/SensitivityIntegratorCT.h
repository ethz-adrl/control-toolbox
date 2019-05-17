/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#pragma once

namespace ct {
namespace optcon {


/**
 * @brief      This class can integrate a controlled system and a costfunction.
 *             Furthermore, it provides first order derivatives with respect to
 *             initial state and control
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 * @tparam     SCALAR       The scalar type
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class SensitivityIntegratorCT
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef ct::core::StateVector<STATE_DIM, SCALAR> state_vector;
    typedef ct::core::ControlVector<CONTROL_DIM, SCALAR> control_vector;
    typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix;
    typedef Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM> state_control_matrix;


    /**
     * @brief      Constructor
     *
     * @param[in]  system       The controlled system
     * @param[in]  stepperType  The integration stepper type
     */
    SensitivityIntegratorCT(const std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& system,
        const ct::core::IntegrationType stepperType = ct::core::IntegrationType::EULERCT)
        : cacheData_(false), cacheSensitivities_(false)
    {
        setControlledSystem(system);
        initializeDerived(stepperType);
    }

    /**
     * @brief      Destroys the object.
     */
    ~SensitivityIntegratorCT() = default;
    /**
     * @brief      Initializes the steppers
     *
     * @param[in]  stepperType  The desired integration stepper type
     */
    void initializeDerived(const ct::core::IntegrationType stepperType)
    {
        switch (stepperType)
        {
            case ct::core::IntegrationType::EULERCT:
            {
                stepperState_ = std::shared_ptr<ct::core::internal::StepperCTBase<state_vector, SCALAR>>(
                    new ct::core::internal::StepperEulerCT<state_vector, SCALAR>());
                stepperDX0_ = std::shared_ptr<ct::core::internal::StepperCTBase<state_matrix, SCALAR>>(
                    new ct::core::internal::StepperEulerCT<state_matrix, SCALAR>());
                stepperDU0_ = std::shared_ptr<ct::core::internal::StepperCTBase<state_control_matrix, SCALAR>>(
                    new ct::core::internal::StepperEulerCT<state_control_matrix, SCALAR>());
                stepperCost_ = std::shared_ptr<ct::core::internal::StepperCTBase<SCALAR, SCALAR>>(
                    new ct::core::internal::StepperEulerCT<SCALAR, SCALAR>());
                stepperCostDX0_ = std::shared_ptr<ct::core::internal::StepperCTBase<state_vector, SCALAR>>(
                    new ct::core::internal::StepperEulerCT<state_vector, SCALAR>());
                stepperCostDU0_ = std::shared_ptr<ct::core::internal::StepperCTBase<control_vector, SCALAR>>(
                    new ct::core::internal::StepperEulerCT<control_vector, SCALAR>());
                break;
            }

            case ct::core::IntegrationType::RK4CT:
            {
                stepperState_ = std::shared_ptr<ct::core::internal::StepperCTBase<state_vector, SCALAR>>(
                    new ct::core::internal::StepperRK4CT<state_vector, SCALAR>());
                stepperDX0_ = std::shared_ptr<ct::core::internal::StepperCTBase<state_matrix, SCALAR>>(
                    new ct::core::internal::StepperRK4CT<state_matrix, SCALAR>());
                stepperDU0_ = std::shared_ptr<ct::core::internal::StepperCTBase<state_control_matrix, SCALAR>>(
                    new ct::core::internal::StepperRK4CT<state_control_matrix, SCALAR>());
                stepperCost_ = std::shared_ptr<ct::core::internal::StepperCTBase<SCALAR, SCALAR>>(
                    new ct::core::internal::StepperRK4CT<SCALAR, SCALAR>());
                stepperCostDX0_ = std::shared_ptr<ct::core::internal::StepperCTBase<state_vector, SCALAR>>(
                    new ct::core::internal::StepperRK4CT<state_vector, SCALAR>());
                stepperCostDU0_ = std::shared_ptr<ct::core::internal::StepperCTBase<control_vector, SCALAR>>(
                    new ct::core::internal::StepperRK4CT<control_vector, SCALAR>());
                break;
            }

            default:
                throw std::runtime_error("Invalid CT integration type");
        }
    }

    /**
     * @brief      Prepares the integrator to provide first order sensitivity
     *             generation by setting a linearsystem, enabling state caching
     *             and settings up the function objects
     *
     * @param[in]  linearSystem  The linearized system
     */
    void setLinearSystem(const std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& linearSystem)
    {
        linearSystem_ = linearSystem;
        cacheData_ = true;

        dX0dot_ = [this](const state_matrix& dX0In, state_matrix& dX0dt, const SCALAR t) {
            if (cacheSensitivities_)
                arraydX0_.push_back(dX0In);

            dX0dt = arrayA_[dX0Index_] * dX0In;
            dX0Index_++;
        };

        dU0dot_ = [this](const state_control_matrix& dU0In, state_control_matrix& dU0dt, const SCALAR t) {
            if (cacheSensitivities_)
                arraydU0_.push_back(dU0In);

            dU0dt = arrayA_[dU0Index_] * dU0In +
                    arrayB_[dU0Index_] *
                        controlledSystem_->getController()->getDerivativeU0(
                            statesCached_[dU0Index_], timesCached_[dU0Index_]);
            dU0Index_++;
        };

        dUfdot_ = [this](const state_control_matrix& dUfIn, state_control_matrix& dUfdt, const SCALAR t) {
            if (cacheSensitivities_)
                arraydUf_.push_back(dUfIn);

            dUfdt = arrayA_[dU0Index_] * dUfIn +
                    arrayB_[dU0Index_] *
                        controlledSystem_->getController()->getDerivativeUf(
                            statesCached_[dU0Index_], timesCached_[dU0Index_]);
            dU0Index_++;
        };
    }

    /**
     * @brief      Changes the controlledsystem to be integrated
     *
     * @param[in]  controlledSystem  The new controlled system
     */
    void setControlledSystem(
        const std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& controlledSystem)
    {
        controlledSystem_ = controlledSystem;
        xDot_ = [this](const state_vector& x, state_vector& dxdt, const SCALAR t) {
            control_vector controlAction;
            controlledSystem_->getController()->computeControl(x, t, controlAction);

            if (cacheData_)
            {
                statesCached_.push_back(x);
                controlsCached_.push_back(controlAction);
                timesCached_.push_back(t);
            }

            controlledSystem_->computeControlledDynamics(x, t, controlAction, dxdt);
        };
    }

    /**
     * @brief      Prepares the integrator to provide cost integration and first
     *             order cost derivatives. This is done by enabling sensitivity
     *             caching and setting up the function objects
     *
     * @param[in]  costFun  The new costfunction
     */
    void setCostFunction(const std::shared_ptr<CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>> costFun)
    {
        costFunction_ = costFun;
        cacheSensitivities_ = true;
        costDot_ = [this](const SCALAR& costIn, SCALAR& cost, const SCALAR t) {
            costFunction_->setCurrentStateAndControl(
                statesCached_[costIndex_], controlsCached_[costIndex_], timesCached_[costIndex_]);
            cost = costFunction_->evaluateIntermediate();
            costIndex_++;
        };

        costdX0dot_ = [this](const state_vector& costdX0In, state_vector& costdX0dt, const SCALAR t) {
            costFunction_->setCurrentStateAndControl(
                statesCached_[costIndex_], controlsCached_[costIndex_], timesCached_[costIndex_]);
            costdX0dt = arraydX0_[costIndex_].transpose() * costFunction_->stateDerivativeIntermediate();
            costIndex_++;
        };

        costdU0dot_ = [this](const control_vector& costdU0In, control_vector& costdU0dt, const SCALAR t) {
            costFunction_->setCurrentStateAndControl(
                statesCached_[costIndex_], controlsCached_[costIndex_], timesCached_[costIndex_]);
            costdU0dt = arraydU0_[costIndex_].transpose() * costFunction_->stateDerivativeIntermediate() +
                        controlledSystem_->getController()->getDerivativeU0(
                            statesCached_[costIndex_], timesCached_[costIndex_]) *
                            costFunction_->controlDerivativeIntermediate();
            costIndex_++;
        };

        costdUfdot_ = [this](const control_vector& costdUfIn, control_vector& costdUfdt, const SCALAR t) {
            costFunction_->setCurrentStateAndControl(
                statesCached_[costIndex_], controlsCached_[costIndex_], timesCached_[costIndex_]);
            costdUfdt = arraydUf_[costIndex_].transpose() * costFunction_->stateDerivativeIntermediate() +
                        controlledSystem_->getController()->getDerivativeUf(
                            statesCached_[costIndex_], timesCached_[costIndex_]) *
                            costFunction_->controlDerivativeIntermediate();
            costIndex_++;
        };
    }

    /**
     * @brief          Integrates the system starting from state and startTime
     *                 for numSteps integration steps. Returns the full state
     *                 and time trajectories
     *
     * @param[in, out] state            The initial state for integration
     * @param[in]      startTime        The start time
     * @param[in]      numSteps         The number steps
     * @param[in]      dt               The integration timestep
     * @param[out]     stateTrajectory  The output state trajectory
     * @param[out]     timeTrajectory   The output time trajectory
     */
    void integrate(state_vector& state,
        const SCALAR startTime,
        const size_t numSteps,
        const SCALAR dt,
        ct::core::StateVectorArray<STATE_DIM, SCALAR>& stateTrajectory,
        ct::core::tpl::TimeArray<SCALAR>& timeTrajectory)
    {
        clearStates();
        clearLinearization();
        cacheData_ = true;
        stateTrajectory.clear();
        timeTrajectory.clear();
        SCALAR time = startTime;
        stateTrajectory.push_back(state);
        timeTrajectory.push_back(time);

        for (size_t i = 0; i < numSteps; ++i)
        {
            stepperState_->do_step(xDot_, state, time, dt);
            time += dt;
            stateTrajectory.push_back(state);
            timeTrajectory.push_back(time);
        }
    }

    /**
     * @brief           Integrates the system starting from state and startTime
     *                  for numSteps integration steps. Returns only the final
     *                  state and time
     *
     * @param[int, out] state      The initial state for integration
     * @param[in]       startTime  The start time
     * @param[in]       numSteps   The number steps
     * @param[in]       dt         The integration timestep
     */
    void integrate(state_vector& state, const SCALAR startTime, const size_t numSteps, const SCALAR dt)
    {
        clearStates();
        clearLinearization();
        SCALAR time = startTime;
        for (size_t i = 0; i < numSteps; ++i)
        {
            stepperState_->do_step(xDot_, state, time, dt);
            time += dt;
        }
    }


    /**
     * @brief          Integrates the sensitivity ODE of the integrator with
     *                 respec to the initial state x0
     *
     * @param[in, out] dX0        The sensitivity matrix wrt x0
     * @param[in]      startTime  The start time
     * @param[in]      numSteps   The number of integration steps
     * @param[in]      dt         The integration timestep
     */
    void integrateSensitivityDX0(state_matrix& dX0, const SCALAR startTime, const size_t numSteps, const SCALAR dt)
    {
        dX0Index_ = 0;
        SCALAR time = startTime;
        dX0.setIdentity();
        for (size_t i = 0; i < numSteps; ++i)
        {
            stepperDX0_->do_step(dX0dot_, dX0, time, dt);
            time += dt;
        }
    }

    /**
     * @brief          Integrates the sensitivity ODE of the integrator with
     *                 respec to the initial control input u0
     *
     * @param[in, out] dU0        The sensitivity matrix wrt u0
     * @param[in]      startTime  The start time
     * @param[in]      numSteps   The number of integration steps
     * @param[in]      dt         The integration timestep
     */
    void integrateSensitivityDU0(state_control_matrix& dU0,
        const SCALAR startTime,
        const size_t numSteps,
        const SCALAR dt)
    {
        dU0Index_ = 0;
        SCALAR time = startTime;
        dU0.setZero();
        for (size_t i = 0; i < numSteps; ++i)
        {
            stepperDU0_->do_step(dU0dot_, dU0, time, dt);
            time += dt;
        }
    }

    /**
     * @brief          Integrates the sensitivity ODE of the integrator with
     *                 respec to the final control input uf
     *
     * @param[in, out] dUF        The sensitivity matrix wrt uF
     * @param[in]      startTime  The start time
     * @param[in]      numSteps   The number of integration steps
     * @param[in]      dt         The integration timestep
     */
    void integrateSensitivityDUf(state_control_matrix& dUf,
        const SCALAR startTime,
        const size_t numSteps,
        const SCALAR dt)
    {
        dU0Index_ = 0;
        SCALAR time = startTime;
        dUf.setZero();
        for (size_t i = 0; i < numSteps; ++i)
        {
            stepperDU0_->do_step(dUfdot_, dUf, time, dt);
            time += dt;
        }
    }

    /**
     * @brief          Integrates the costfunction using the states and controls
     *                 from the costintegration
     *
     * @param[in, out] cost       The initial cost
     * @param[in]      startTime  The start time
     * @param[in]      numSteps   The number of integration steps
     * @param[in]      dt         The integration time step
     */
    void integrateCost(SCALAR& cost, const SCALAR startTime, const size_t numSteps, const SCALAR dt)
    {
        SCALAR time = startTime;
        costIndex_ = 0;
        if (statesCached_.size() == 0 || controlsCached_.size() == 0 || timesCached_.size() == 0)
            throw std::runtime_error("States cached are empty");

        for (size_t i = 0; i < numSteps; ++i)
        {
            stepperCost_->do_step(costDot_, cost, time, dt);
            time += dt;
        }
    }


    /**
     * @brief          Integrates the sensitivity of the cost with respect to
     *                 the initial state x0
     *
     * @param[in, out] dX0        The initial cost sensitivity vector
     * @param[in]      startTime  The start time
     * @param[in]      numSteps   The number of integration steps
     * @param[in]      dt         The integration time step
     */
    void integrateCostSensitivityDX0(state_vector& dX0, const SCALAR startTime, const size_t numSteps, const SCALAR dt)
    {
        costIndex_ = 0;
        SCALAR time = startTime;
        dX0.setZero();
        for (size_t i = 0; i < numSteps; ++i)
        {
            stepperCostDX0_->do_step(costdX0dot_, dX0, time, dt);
            time += dt;
        }
    }

    /**
     * @brief          Integrates the sensitivity of the cost with respect to
     *                 the initial control input u0
     *
     * @param[in, out] dU0        The initial cost sensitivity vector
     * @param[in]      startTime  The start time
     * @param[in]      numSteps   The number of integration steps
     * @param[in]      dt         The integration time step
     */
    void integrateCostSensitivityDU0(control_vector& dU0,
        const SCALAR startTime,
        const size_t numSteps,
        const SCALAR dt)
    {
        costIndex_ = 0;
        SCALAR time = startTime;
        dU0.setZero();
        for (size_t i = 0; i < numSteps; ++i)
        {
            stepperCostDU0_->do_step(costdU0dot_, dU0, time, dt);
            time += dt;
        }
    }

    /**
     * @brief          Integrates the sensitivity of the cost with respect to
     *                 the final control input uF
     *
     * @param[in, out] dUf        The initial cost sensitivity vector
     * @param[in]      startTime  The start time
     * @param[in]      numSteps   The number of integration steps
     * @param[in]      dt         The integration time step
     */
    void integrateCostSensitivityDUf(control_vector& dUf,
        const SCALAR& startTime,
        const size_t numSteps,
        const SCALAR dt)
    {
        costIndex_ = 0;
        SCALAR time = startTime;
        dUf.setZero();
        for (size_t i = 0; i < numSteps; ++i)
        {
            stepperCostDU0_->do_step(costdUfdot_, dUf, time, dt);
            time += dt;
        }
    }

    /**
     * @brief      Linearizes the system around the rollout from the state
     *             interation
     */
    void linearize()
    {
        for (size_t i = 0; i < statesCached_.size(); ++i)
        {
            arrayA_.push_back(linearSystem_->getDerivativeState(statesCached_[i], controlsCached_[i], timesCached_[i]));
            arrayB_.push_back(
                linearSystem_->getDerivativeControl(statesCached_[i], controlsCached_[i], timesCached_[i]));
        }
    }

    /**
     * @brief      Clears the cached states, controls and times
     */
    void clearStates()
    {
        statesCached_.clear();
        controlsCached_.clear();
        timesCached_.clear();
    }


    /**
     * @brief      Clears the cached sensitivities
     */
    void clearSensitivities()
    {
        arraydX0_.clear();
        arraydU0_.clear();
        arraydUf_.clear();
    }


    /**
     * @brief      Clears the linearized matrices
     */
    void clearLinearization()
    {
        arrayA_.clear();
        arrayB_.clear();
    }

private:
    std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> controlledSystem_;
    std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>> linearSystem_;
    std::shared_ptr<optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>> costFunction_;

    // Integrate the function
    std::function<void(const SCALAR, SCALAR&, const SCALAR)> costDot_;
    std::function<void(const state_vector&, state_vector&, const SCALAR)> costdX0dot_;
    std::function<void(const control_vector&, control_vector&, const SCALAR)> costdU0dot_;
    std::function<void(const control_vector&, control_vector&, const SCALAR)> costdUfdot_;

    // Sensitivities
    std::function<void(const state_matrix&, state_matrix&, const SCALAR)> dX0dot_;
    std::function<void(const state_control_matrix&, state_control_matrix&, const SCALAR)> dU0dot_;
    std::function<void(const state_control_matrix&, state_control_matrix&, const SCALAR)> dUfdot_;

    // Cache
    bool cacheData_;
    bool cacheSensitivities_;
    ct::core::StateVectorArray<STATE_DIM, SCALAR> statesCached_;
    ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> controlsCached_;
    ct::core::tpl::TimeArray<SCALAR> timesCached_;

    ct::core::StateMatrixArray<STATE_DIM, SCALAR> arrayA_;
    ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR> arrayB_;

    ct::core::StateMatrixArray<STATE_DIM, SCALAR> arraydX0_;
    ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR> arraydU0_;
    ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR> arraydUf_;

    std::shared_ptr<ct::core::internal::StepperCTBase<state_vector, SCALAR>> stepperState_;
    std::shared_ptr<ct::core::internal::StepperCTBase<state_matrix, SCALAR>> stepperDX0_;
    std::shared_ptr<ct::core::internal::StepperCTBase<state_control_matrix, SCALAR>> stepperDU0_;

    std::shared_ptr<ct::core::internal::StepperCTBase<SCALAR, SCALAR>> stepperCost_;
    std::shared_ptr<ct::core::internal::StepperCTBase<state_vector, SCALAR>> stepperCostDX0_;
    std::shared_ptr<ct::core::internal::StepperCTBase<control_vector, SCALAR>> stepperCostDU0_;

    size_t costIndex_;
    size_t dX0Index_;
    size_t dU0Index_;

    std::function<void(const state_vector&, state_vector&, const SCALAR)> xDot_;
};
}
}
