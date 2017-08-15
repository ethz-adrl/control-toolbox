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


#ifndef CT_CORE_SIMPLE_SENSITIVITY_INTEGRATOR_CT_H_
#define CT_CORE_SIMPLE_SENSITIVITY_INTEGRATOR_CT_H_

#include <ct/core/systems/ControlledSystem.h>
#include <ct/core/systems/linear/LinearSystem.h>
#include <ct/core/integration/internal/SteppersCT.h>

namespace ct {
namespace core{


/**
 * @brief      This class can integrate a controlled system
 *             Furthermore, it provides first order derivatives with respect to
 *             initial state and control
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 * @tparam     SCALAR       The scalar type
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class SimpleSensitivityIntegratorCT
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
    SimpleSensitivityIntegratorCT(
        const std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR> >& system,
        const ct::core::IntegrationType stepperType = ct::core::IntegrationType::EULERCT)
    :
    cacheData_(false),
    cacheSensitivities_(false)
    {
        setControlledSystem(system);
        initializeDerived(stepperType);
    }


    SimpleSensitivityIntegratorCT(
        const std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR> >& system,
		const std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& linearSystem,
        const ct::core::IntegrationType stepperType = ct::core::IntegrationType::EULERCT)
    :
    cacheData_(false),
    cacheSensitivities_(false)
    {
        setControlledSystem(system);
        setLinearSystem(linearSystem);
        initializeDerived(stepperType);
    }


    /**
     * @brief      Destroys the object.
     */
    ~SimpleSensitivityIntegratorCT(){}

    /**
     * @brief      Initializes the steppers
     *
     * @param[in]  stepperType  The desired integration stepper type
     */
    void initializeDerived(const ct::core::IntegrationType stepperType)
    {
        switch(stepperType)
        {
            case ct::core::IntegrationType::EULERCT:
            {
                stepperState_ = std::shared_ptr<ct::core::internal::StepperCTBase<state_vector, SCALAR>>(
                    new ct::core::internal::StepperEulerCT<state_vector, SCALAR>());
                stepperDX0_ = std::shared_ptr<ct::core::internal::StepperCTBase<state_matrix, SCALAR>>(
                    new ct::core::internal::StepperEulerCT<state_matrix, SCALAR>());
                stepperDU0_ = std::shared_ptr<ct::core::internal::StepperCTBase<state_control_matrix, SCALAR>>(
                    new ct::core::internal::StepperEulerCT<state_control_matrix, SCALAR>());
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
        
        dX0dot_ = [this](const state_matrix& dX0In, state_matrix& dX0dt, const SCALAR t){
            if(cacheSensitivities_)
                arraydX0_.push_back(dX0In);

            dX0dt = arrayA_[dX0Index_] * dX0In;
            dX0Index_++;
        };

        dU0dot_ = [this](const state_control_matrix& dU0In, state_control_matrix& dU0dt, const SCALAR t){
            if(cacheSensitivities_)
                arraydU0_.push_back(dU0In);

            dU0dt = arrayA_[dU0Index_] * dU0In + 
                    arrayB_[dU0Index_] * controlledSystem_->getController()->getDerivativeU0(statesCached_[dU0Index_], timesCached_[dU0Index_]);
            dU0Index_++;                                                                                  
        };
    }

    /**
     * @brief      Changes the controlledsystem to be integrated
     *
     * @param[in]  controlledSystem  The new controlled system
     */
    void setControlledSystem(const std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& controlledSystem)
    {
        controlledSystem_ = controlledSystem;
        xDot_ = [this](const state_vector& x, state_vector& dxdt, const SCALAR t) {
            control_vector controlAction;
            controlledSystem_->getController()->computeControl(x, t, controlAction);

            if(cacheData_)
            {
                statesCached_.push_back(x);
                controlsCached_.push_back(controlAction);
                timesCached_.push_back(t);
            }

            controlledSystem_->computeControlledDynamics(x, t, controlAction, dxdt);
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
    void integrate_n_steps(
            state_vector& state,
            const SCALAR startTime,
            const size_t numSteps,
            const SCALAR dt,
            ct::core::StateVectorArray<STATE_DIM, SCALAR>& stateTrajectory,
            ct::core::tpl::TimeArray<SCALAR>& timeTrajectory
    )
    {
        clearStates();
        clearLinearization();
        cacheData_ = true;
        stateTrajectory.clear();
        timeTrajectory.clear();
        SCALAR time = startTime;
        stateTrajectory.push_back(state);
        timeTrajectory.push_back(time);

        for(size_t i = 0; i < numSteps; ++i)
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
    void integrate_n_steps(
            state_vector& state,
            const SCALAR startTime,
            const size_t numSteps,
            const SCALAR dt
    )
    {
        clearStates();
        clearLinearization();
        SCALAR time = startTime;
        for(size_t i = 0; i < numSteps; ++i)
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
    void integrateSensitivityDX0(
        state_matrix& dX0,
        const SCALAR startTime,
        const size_t numSteps,
        const SCALAR dt
        )
    {
        dX0Index_ = 0;
        SCALAR time = startTime;
        dX0.setIdentity();
        for(size_t i = 0; i < numSteps; ++i)
        {
            stepperDX0_->do_step(dX0dot_, dX0, time, dt);
            time += dt;
        }
    }

    /**
     * @brief          Integrates the sensitivity ODE of the integrator with
     *                 respect to the initial control input u0
     *
     * @param[in, out] dU0        The sensitivity matrix wrt u0
     * @param[in]      startTime  The start time
     * @param[in]      numSteps   The number of integration steps
     * @param[in]      dt         The integration timestep
     */
    void integrateSensitivityDU0(
        state_control_matrix& dU0,
        const SCALAR startTime,
        const size_t numSteps,
        const SCALAR dt
        )
    {
        dU0Index_ = 0;
        SCALAR time = startTime;
        dU0.setZero();
        for(size_t i = 0; i < numSteps; ++i)
        {
            stepperDU0_->do_step(dU0dot_, dU0, time, dt);
            time += dt;
        }
    }


    /**
     * @brief      Linearizes the system around the rollout from the state
     *             interation
     */
    void linearize()
    {
        for(size_t i = 0; i < statesCached_.size(); ++i)
        {
            arrayA_.push_back(linearSystem_->getDerivativeState(statesCached_[i], controlsCached_[i], timesCached_[i]));
            arrayB_.push_back(linearSystem_->getDerivativeControl(statesCached_[i], controlsCached_[i], timesCached_[i]));
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
    }


    /**
     * @brief      Clears the linearized matrices
     */
    void clearLinearization()
    {
        arrayA_.clear();
        arrayB_.clear();
    }

    void clearAll(){
    	clearStates();
    	clearSensitivities();
    	clearLinearization();
    }

private:

    std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> controlledSystem_;
    std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR> > linearSystem_;

    // Sensitivities
    std::function<void (const state_matrix&, state_matrix&, const SCALAR)> dX0dot_;
    std::function<void (const state_control_matrix&, state_control_matrix&, const SCALAR)> dU0dot_;

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

    std::shared_ptr<ct::core::internal::StepperCTBase<state_vector, SCALAR>> stepperState_;
    std::shared_ptr<ct::core::internal::StepperCTBase<state_matrix, SCALAR>> stepperDX0_;
    std::shared_ptr<ct::core::internal::StepperCTBase<state_control_matrix, SCALAR>> stepperDU0_;

    size_t dX0Index_;
    size_t dU0Index_;

    std::function<void (const state_vector&, state_vector&, const SCALAR)> xDot_;


};


}
}

#endif
