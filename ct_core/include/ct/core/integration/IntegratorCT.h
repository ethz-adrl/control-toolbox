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

enum IntegrationTypeCT
{
    EULER, RK4
};


template <size_t STATE_DIM, typename SCALAR = double>
class IntegratorCT
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef StateVector<STATE_DIM, SCALAR> state_vector;


    IntegratorCT(
        const std::shared_ptr<System<STATE_DIM, SCALAR> >& system,
        const IntegrationTypeCT stepperType = IntegrationTypeCT::EULER)
    {
        setNonlinearSystem(system);
        initialize(stepperType);
    }

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

    void setNonlinearSystem(const std::shared_ptr<System<STATE_DIM, SCALAR>>& system)
    {
        system_ = system;
        xDot_ = [this](const state_vector& x, const SCALAR t, state_vector& dxdt) {
            system_->computeDynamics(x, t, dxdt);
        };       
    }

    virtual void integrate(
            state_vector& state,
            const SCALAR& startTime,
            size_t numSteps,
            SCALAR dt,
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

    virtual void integrate(
            state_vector& state,
            const SCALAR& startTime,
            size_t numSteps,
            SCALAR dt
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
