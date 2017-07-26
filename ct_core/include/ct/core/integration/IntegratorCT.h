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


#ifndef CT_CORE_INTEGRATOR_CT_H_
#define CT_CORE_INTEGRATOR_CT_H_


#include <ct/core/types/trajectories/TimeArray.h>
#include <ct/core/types/trajectories/StateVectorArray.h>
#include <ct/core/systems/System.h>
#include <ct/core/integration/internal/SteppersCT.h>

namespace ct {
namespace core {

/**
 * @brief      Enum for the stepper type
 */
enum IntegrationTypeCT
{
    EULER, RK4
};


/**
 * @brief      Class for integrating a system. For now this class only
 *             implements fixed size steppers.
 *
 * @tparam     STATE_DIM  The state dimension
 * @tparam     SCALAR     The scalar type
 */
template <size_t STATE_DIM, typename SCALAR = double>
class IntegratorCT
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef StateVector<STATE_DIM, SCALAR> state_vector;


    /**
     * @brief      Constructor
     *
     * @param[in]  system       The system to be integrated
     * @param[in]  stepperType  The stepper type
     */
    IntegratorCT(
        const std::shared_ptr<System<STATE_DIM, SCALAR> >& system,
        const IntegrationTypeCT stepperType = IntegrationTypeCT::EULER)
    {
        setNonlinearSystem(system);
        initialize(stepperType);
    }

    /**
     * @brief      Initializes the integrator, can be used to switch between stepper types
     *
     * @param[in]  stepperType  The stepper type
     */
    void initialize(const IntegrationTypeCT stepperType)
    {
        switch(stepperType)
        {
            case EULER:
            {
                stepperState_ = std::shared_ptr<internal::StepperBaseCT<SCALAR, state_vector>>(
                    new internal::StepperEulerCT<SCALAR, state_vector>());
                break;
            }

            case RK4:
            {
                stepperState_ = std::shared_ptr<internal::StepperBaseCT<SCALAR, state_vector>>(
                    new internal::StepperRK4CT<SCALAR, state_vector>());
                break;
            }

            default:
                throw std::runtime_error("Invalid CT integration type");
        }
    }

    /**
     * @brief      Sets the system to be integrated, can be used to change the system
     *
     * @param[in]  system  The system
     */
    void setNonlinearSystem(const std::shared_ptr<System<STATE_DIM, SCALAR>>& system)
    {
        system_ = system;
        xDot_ = [this](const state_vector& x, const SCALAR t, state_vector& dxdt) {
            system_->computeDynamics(x, t, dxdt);
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
    void integrate(
            state_vector& state,
            const SCALAR& startTime,
            const size_t numSteps,
            const SCALAR dt,
            StateVectorArray<STATE_DIM, SCALAR>& stateTrajectory,
            tpl::TimeArray<SCALAR>& timeTrajectory
    )
    {
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
    void integrate(
            state_vector& state,
            const SCALAR& startTime,
            const size_t numSteps,
            const SCALAR dt
    )
    {
        SCALAR time = startTime;
        for(size_t i = 0; i < numSteps; ++i)
        {
            stepperState_->do_step(xDot_, state, time, dt);
            time += dt;
        }
    }


protected:
    std::shared_ptr<System<STATE_DIM, SCALAR> > system_; //! pointer to the system                                                   

    // Integrate the function
    std::function<void (const state_vector&, const SCALAR, state_vector&)> xDot_;

    std::shared_ptr<internal::StepperBaseCT<SCALAR, state_vector>> stepperState_;
};

}
}

#endif
