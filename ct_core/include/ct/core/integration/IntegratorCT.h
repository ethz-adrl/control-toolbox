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



template <size_t STATE_DIM, class Stepper, typename SCALAR = double>
class IntegratorCT
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    IntegratorCT(const std::shared_ptr<System<STATE_DIM, SCALAR> >& system) :
    system_(system)
    {

    }

    void integrate(
            StateVector<STATE_DIM, SCALAR>& state,
            const SCALAR& startTime,
            size_t numSteps,
            SCALAR dt,
            StateVectorArray<STATE_DIM, SCALAR>& stateTrajectory,
            tpl::TimeArray<SCALAR>& timeTrajectory
    )
    {

    }

    void integrate(
            StateVector<STATE_DIM, SCALAR>& state,
            const SCALAR& startTime,
            size_t numSteps,
            SCALAR dt
    )
    {
        SCALAR time = startTime;
        StateVector<STATE_DIM, SCALAR> derivate;

        for(size_t i = 0; i < numSteps; ++i)
        {
            system_->computeDynamics(state, time, derivate);
            stepper_.do_step(derivate, state, time, dt);
            time += dt;
        }
    }


private:
    std::shared_ptr<System<STATE_DIM, SCALAR> > system_; //! pointer to the system
    Stepper stepper_;


};


template <size_t STATE_DIM, typename SCALAR = double>
using IntegratorEulerCT = IntegratorCT<STATE_DIM, internal::StepperEulerCT<STATE_DIM, SCALAR>, SCALAR>;

}
}

#endif
