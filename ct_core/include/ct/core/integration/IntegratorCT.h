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
#include <ct/core/systems/linear/LinearSystem.h>
#include <ct/core/integration/internal/SteppersCT.h>

namespace ct {
namespace core {



template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class IntegratorCT
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef StateVector<STATE_DIM, SCALAR> state_vector;
    typedef ControlVector<CONTROL_DIM, SCALAR> control_vector;
    typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix;
    typedef Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM> state_control_matrix;

    // Need a solution to switch between steppers...
    template <typename MatrixType>
    using StepperEuler = internal::StepperEulerCT<SCALAR, MatrixType>;

    IntegratorCT(const std::shared_ptr<System<STATE_DIM, SCALAR> >& system)
    :
    cacheData_(false)
    {
        setNonlinearSystem(system);
    }

    // or prepare for sensitivity integration...
    void setLinearSystem(const std::shared_ptr<LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& linearSystem)
    {
        linearSystem_ = linearSystem;
        cacheData_ = true;
        // Need to cache the derivatives and determine the correct indices (maybe switch to the trajectories....)
        dX0dot_ = [this](const state_matrix& dX0dtIn, const SCALAR t, state_matrix& dX0dt){
            dX0dt = arrayA_.front() * dX0dtIn;
        };

        dU0dot_ = [this](const state_control_matrix& dU0dtIn, const SCALAR t, state_control_matrix& dU0dt){
            dU0dt = arrayA_.front() * dU0dtIn + arrayB_.front() * control_matrix::Identity(); // dudU0 need to figure out how to get this derivative
        }; 
    }

    void setNonlinearSystem(const std::shared_ptr<System<STATE_DIM, SCALAR>>& system)
    {
        system_ = system;
        xDot_ = [this](const state_vector& x, const SCALAR t, state_vector& dxdt) {
            this->system_->computeDynamics(x, t, dxdt);
            if(cacheData_)
            {
                statesCached_.push_back(x);
                controlsCached_.push_back(control_vector::Zero());
                timesCached_.push_back(t);
            }
        };       
    }

    void integrate(
            state_vector& state,
            const SCALAR& startTime,
            size_t numSteps,
            SCALAR dt,
            StateVectorArray<STATE_DIM, SCALAR>& stateTrajectory,
            tpl::TimeArray<SCALAR>& timeTrajectory
    )
    {
        StepperEuler<state_vector> stepper;
        clearCache();
        stateTrajectory.clear();
        timeTrajectory.clear();
        SCALAR time = startTime;
        stateTrajectory.push_back(state);
        timeTrajectory.push_back(time);

        for(size_t i = 0; i < numSteps; ++i)
        {
            stepper.do_step(xDot_, state, time, dt);
            time += dt;
            stateTrajectory.push_back(state);
            timeTrajectory.push_back(time);
        }
    }

    void integrate(
            state_vector& state,
            const SCALAR& startTime,
            size_t numSteps,
            SCALAR dt
    )
    {
        StepperEuler<state_vector> stepper;
        clearCache();
        SCALAR time = startTime;
        for(size_t i = 0; i < numSteps; ++i)
        {
            stepper.do_step(xDot_, state, time, dt);
            time += dt;
        }
    }

    // Need some a method to ensure no double caching 
    void integrateSensitivityDX0(
        state_matrix& dX0,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt
        )
    {
        StepperEuler<state_matrix> stepper;
        clearMatrixCache();
        cacheA();
        cacheB();
        SCALAR time = startTime;
        dX0.setIdentity();
        for(size_t i = 0; i < numSteps; ++i)
        {
            stepper.do_step(dX0dot_, dX0, time, dt);
            time += dt;
        }
    }

    void integrateSensitivityDU0(
        state_control_matrix& dU0,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt
        )
    {
        StepperEuler<state_matrix> stepper;
        // clearMatrixCache();
        // cacheA();
        // cacheB();
        SCALAR time = startTime;
        dU0.setZero();
        for(size_t i = 0; i < numSteps; ++i)
        {
            stepper.do_step(dU0dot_, dU0, time, dt);
            time += dt;
        }
    }


private:
    void clearCache()
    {
        statesCached_.clear();
        controlsCached_.clear();
        timesCached_.clear();
    }

    void cacheA()
    {
        arrayA_.push_back(linearSystem_->getDerivativeState(statesCached_.front(), controlsCached_.front(), timesCached_.front()));
    }

    void cacheB()
    {
        arrayB_.push_back(linearSystem_->getDerivativeControl(statesCached_.front(), controlsCached_.front(), timesCached_.front()));
    }

    void clearMatrixCache()
    {
        arrayA_.clear();
        arrayB_.clear();
    }

    std::shared_ptr<System<STATE_DIM, SCALAR> > system_; //! pointer to the system 
    std::shared_ptr<LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR> > linearSystem_;                                                     

    // Integrate the function
    std::function<void (const state_vector&, const SCALAR, state_vector&)> xDot_;

    // Sensitivities
    std::function<void (const state_matrix&, const SCALAR, state_matrix&)> dX0dot_;
    std::function<void (const state_control_matrix&, const SCALAR, state_control_matrix&)> dU0dot_;

    // Cache
    bool cacheData_;
    StateVectorArray<STATE_DIM, SCALAR> statesCached_;
    ControlVectorArray<CONTROL_DIM, SCALAR> controlsCached_;
    tpl::TimeArray<SCALAR> timesCached_;
    StateMatrixArray<STATE_DIM, SCALAR> arrayA_;
    StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR> arrayB_;
};


// template <size_t STATE_DIM, typename SCALAR = double>
// using IntegratorEulerCT = IntegratorCT<STATE_DIM, internal::StepperEulerCT<SCALAR, Eigen::Matrix<double, STATE_DIM, 1>>, SCALAR>;

// template <size_t STATE_DIM, typename SCALAR = double>
// using IntegratorRK4CT = IntegratorCT<STATE_DIM, internal::StepperRK4CT<SCALAR, Eigen::Matrix<double, STATE_DIM, 1>>, SCALAR>;

}
}

#endif
