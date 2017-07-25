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


#ifndef CT_CORE_SENSITIVITY_INTEGRATOR_CT_H_
#define CT_CORE_SENSITIVITY_INTEGRATOR_CT_H_

#include <ct/core/systems/ControlledSystem.h>
#include <ct/core/systems/linear/LinearSystem.h>
#include <ct/core/integration/IntegratorCT.h>

namespace ct {
namespace core {


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class SensitivityIntegratorCT
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef StateVector<STATE_DIM, SCALAR> state_vector;
    typedef ControlVector<CONTROL_DIM, SCALAR> control_vector;
    typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix;
    typedef Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM> state_control_matrix;
    // typedef IntegratorCT<STATE_DIM, SCALAR> BASE;


    SensitivityIntegratorCT(
        const std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR> >& system,
        const IntegrationTypeCT stepperType = IntegrationTypeCT::EULER)
    :
    // BASE(system),
    cacheData_(false),
    cacheSensitivities_(false)
    {
        setControlledSystem(system);
        initializeDerived(stepperType);
    }

    virtual ~SensitivityIntegratorCT(){}

    void initializeDerived(const IntegrationTypeCT stepperType)
    {
        switch(stepperType)
        {
            case EULER:
            {
                stepperState_ = std::shared_ptr<internal::StepperBaseCT<SCALAR, state_vector>>(
                    new internal::StepperEulerCT<SCALAR, state_vector>());
                stepperDX0_ = std::shared_ptr<internal::StepperBaseCT<SCALAR, state_matrix>>(
                    new internal::StepperEulerCT<SCALAR, state_matrix>());
                stepperDU0_ = std::shared_ptr<internal::StepperBaseCT<SCALAR, state_control_matrix>>(
                    new internal::StepperEulerCT<SCALAR, state_control_matrix>());
                stepperCost_ = std::shared_ptr<internal::StepperBaseCT<SCALAR, SCALAR>>(
                    new internal::StepperEulerCT<SCALAR, SCALAR>());
                stepperCostDX0_ = std::shared_ptr<internal::StepperBaseCT<SCALAR, state_vector>>(
                    new internal::StepperEulerCT<SCALAR, state_vector>());
                stepperCostDU0_ = std::shared_ptr<internal::StepperBaseCT<SCALAR, control_vector>>(
                    new internal::StepperEulerCT<SCALAR, control_vector>());
                break;
            }

            case RK4:
            {
                stepperState_ = std::shared_ptr<internal::StepperBaseCT<SCALAR, state_vector>>(
                    new internal::StepperRK4CT<SCALAR, state_vector>());
                stepperDX0_ = std::shared_ptr<internal::StepperBaseCT<SCALAR, state_matrix>>(
                    new internal::StepperRK4CT<SCALAR, state_matrix>());
                stepperDU0_ = std::shared_ptr<internal::StepperBaseCT<SCALAR, state_control_matrix>>(
                    new internal::StepperRK4CT<SCALAR, state_control_matrix>());
                stepperCost_ = std::shared_ptr<internal::StepperBaseCT<SCALAR, SCALAR>>(
                    new internal::StepperRK4CT<SCALAR, SCALAR>());
                stepperCostDX0_ = std::shared_ptr<internal::StepperBaseCT<SCALAR, state_vector>>(
                    new internal::StepperRK4CT<SCALAR, state_vector>());
                stepperCostDU0_ = std::shared_ptr<internal::StepperBaseCT<SCALAR, control_vector>>(
                    new internal::StepperRK4CT<SCALAR, control_vector>());
                break;
            }

            default:
                throw std::runtime_error("Invalid CT integration type");
        }
    }

    // or prepare for sensitivity integration...
    void setLinearSystem(const std::shared_ptr<LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& linearSystem)
    {
        linearSystem_ = linearSystem;
        cacheData_ = true;
        // Need to cache the derivatives and determine the correct indices (maybe switch to the trajectories....)
        dX0dot_ = [this](const state_matrix& dX0In, const SCALAR t, state_matrix& dX0dt){
            if(cacheSensitivities_)
                arraydX0_.push_back(dX0In);

            dX0dt = arrayA_[dX0Index_] * dX0In;
            dX0Index_++;
        };

        dU0dot_ = [this](const state_control_matrix& dU0In, const SCALAR t, state_control_matrix& dU0dt){
            if(cacheSensitivities_)
                arraydU0_.push_back(dU0In);

            dU0dt = arrayA_[dU0Index_] * dU0In + 
                    arrayB_[dU0Index_] * controlledSystem_->getController()->getDerivativeU0(statesCached_[dU0Index_], timesCached_[dU0Index_]);
            dU0Index_++;                                                                                  
        };

        dUfdot_ =  [this](const state_control_matrix& dUfIn, const SCALAR t, state_control_matrix& dUfdt){
            if(cacheSensitivities_)
                arraydUf_.push_back(dUfIn);

            dUfdt = arrayA_[dU0Index_] * dUfIn + 
                    arrayB_[dU0Index_] * controlledSystem_->getController()->getDerivativeUf(statesCached_[dU0Index_], timesCached_[dU0Index_]);
            dU0Index_++;                                                                                  
        };
    }

    void setControlledSystem(const std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& controlledSystem)
    {
        controlledSystem_ = controlledSystem;
        xDot_ = [this](const state_vector& x, const SCALAR t, state_vector& dxdt) {
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

    void setCostFunction(const std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>> costFun)
    {
        costFunction_ = costFun;
        cacheSensitivities_ = true;
        costDot_ = [this](const SCALAR& costIn, const SCALAR t, SCALAR& cost)
        {
            costFunction_->setCurrentStateAndControl(statesCached_[costIndex_], controlsCached_[costIndex_], timesCached_[costIndex_]);
            cost = costFunction_->evaluateIntermediate();
            costIndex_++;
        };

        costdX0dot_ = [this](const state_vector& costdX0In, const SCALAR t, state_vector& costdX0dt){
            costFunction_->setCurrentStateAndControl(statesCached_[costIndex_], controlsCached_[costIndex_], timesCached_[costIndex_]);
            costdX0dt = arraydX0_[costIndex_].transpose() * costFunction_->stateDerivativeIntermediate();
            costIndex_++;
        };

        costdU0dot_ = [this](const control_vector& costdU0In, const SCALAR t, control_vector& costdU0dt){
            costFunction_->setCurrentStateAndControl(statesCached_[costIndex_], controlsCached_[costIndex_], timesCached_[costIndex_]);
            costdU0dt = arraydU0_[costIndex_].transpose() * costFunction_->stateDerivativeIntermediate() + 
                        controlledSystem_->getController()->getDerivativeU0(statesCached_[costIndex_], timesCached_[costIndex_]) * 
                        costFunction_->controlDerivativeIntermediate();
            costIndex_++;
        };

        costdUfdot_ = [this](const control_vector& costdUfIn, const SCALAR t, control_vector& costdUfdt){
            costFunction_->setCurrentStateAndControl(statesCached_[costIndex_], controlsCached_[costIndex_], timesCached_[costIndex_]);
            costdUfdt = arraydUf_[costIndex_].transpose() * costFunction_->stateDerivativeIntermediate() + 
                        controlledSystem_->getController()->getDerivativeUf(statesCached_[costIndex_], timesCached_[costIndex_]) *
                        costFunction_->controlDerivativeIntermediate();
            costIndex_++;
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

    void integrate(
            state_vector& state,
            const SCALAR& startTime,
            size_t numSteps,
            SCALAR dt
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

    // Need some a method to ensure no double caching 
    void integrateSensitivityDX0(
        state_matrix& dX0,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt
        )
    {
        // clearSensitivities();
        dX0Index_ = 0;
        SCALAR time = startTime;
        dX0.setIdentity();
        for(size_t i = 0; i < numSteps; ++i)
        {
            stepperDX0_->do_step(dX0dot_, dX0, time, dt);
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
        // clearSensitivities();
        dU0Index_ = 0;
        SCALAR time = startTime;
        dU0.setZero();
        for(size_t i = 0; i < numSteps; ++i)
        {
            stepperDU0_->do_step(dU0dot_, dU0, time, dt);
            time += dt;
        }
    }

    void integrateSensitivityDUf(
        state_control_matrix& dUf,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt
        )
    {
        // clearSensitivities();
        dU0Index_ = 0;
        SCALAR time = startTime;
        dUf.setZero();
        for(size_t i = 0; i < numSteps; ++i)
        {
            stepperDU0_->do_step(dUfdot_, dUf, time, dt);
            time += dt;
        }
    }

    void integrateCost(
        SCALAR& cost,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt)
    {
        SCALAR time = startTime;
        costIndex_ = 0;
        if( statesCached_.size() == 0 ||
            controlsCached_.size() == 0 ||
            timesCached_.size() == 0)
                throw std::runtime_error("States cached are empty");

        for(size_t i = 0; i < numSteps; ++i)
        {
            stepperCost_->do_step(costDot_, cost, time, dt);
            time += dt;
        }
    }


    void integrateCostSensitivityDX0(
        state_vector& dX0,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt
        )
    {
        // clearSensitivities();
        costIndex_ = 0;
        SCALAR time = startTime;
        dX0.setZero();
        for(size_t i = 0; i < numSteps; ++i)
        {
            stepperCostDX0_->do_step(costdX0dot_, dX0, time, dt);
            time += dt;
        }
    }

    void integrateCostSensitivityDU0(
        control_vector& dU0,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt
        )
    {
        // clearSensitivities();
        costIndex_ = 0;
        SCALAR time = startTime;
        dU0.setZero();
        for(size_t i = 0; i < numSteps; ++i)
        {
            stepperCostDU0_->do_step(costdU0dot_, dU0, time, dt);
            time += dt;
        }
    }

    void integrateCostSensitivityDUf(
        control_vector& dUf,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt
        )
    {
        // clearSensitivities();
        costIndex_ = 0;
        SCALAR time = startTime;
        dUf.setZero();
        for(size_t i = 0; i < numSteps; ++i)
        {
            stepperCostDU0_->do_step(costdUfdot_, dUf, time, dt);
            time += dt;
        }
    }

    void linearize()
    {
        for(size_t i = 0; i < statesCached_.size(); ++i)
        {
            arrayA_.push_back(linearSystem_->getDerivativeState(statesCached_[i], controlsCached_[i], timesCached_[i]));
            arrayB_.push_back(linearSystem_->getDerivativeControl(statesCached_[i], controlsCached_[i], timesCached_[i]));
        }
    }    

    void clearStates()
    {
        statesCached_.clear();
        controlsCached_.clear();
        timesCached_.clear();
    }


    void clearSensitivities()
    {
        arraydX0_.clear();
        arraydU0_.clear();
        arraydUf_.clear();
    }


    void clearLinearization()
    {
        arrayA_.clear();
        arrayB_.clear();
    }

private:

    std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> controlledSystem_;
    std::shared_ptr<LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR> > linearSystem_;
    std::shared_ptr<optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR> > costFunction_;                                                     

    // Integrate the function
    std::function<void (const SCALAR, const SCALAR, SCALAR&)> costDot_;
    std::function<void (const state_vector&, const SCALAR, state_vector&)> costdX0dot_;
    std::function<void (const control_vector&, const SCALAR, control_vector&)> costdU0dot_;
    std::function<void (const control_vector&, const SCALAR, control_vector&)> costdUfdot_;

    // Sensitivities
    std::function<void (const state_matrix&, const SCALAR, state_matrix&)> dX0dot_;
    std::function<void (const state_control_matrix&, const SCALAR, state_control_matrix&)> dU0dot_;
    std::function<void (const state_control_matrix&, const SCALAR, state_control_matrix&)> dUfdot_;

    // Cache
    bool cacheData_;
    bool cacheSensitivities_;
    StateVectorArray<STATE_DIM, SCALAR> statesCached_;
    ControlVectorArray<CONTROL_DIM, SCALAR> controlsCached_;
    tpl::TimeArray<SCALAR> timesCached_;

    StateMatrixArray<STATE_DIM, SCALAR> arrayA_;
    StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR> arrayB_;

    StateMatrixArray<STATE_DIM, SCALAR> arraydX0_;
    StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR> arraydU0_;
    StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR> arraydUf_;

    std::shared_ptr<internal::StepperBaseCT<SCALAR, state_matrix>> stepperDX0_;
    std::shared_ptr<internal::StepperBaseCT<SCALAR, state_control_matrix>> stepperDU0_;

    std::shared_ptr<internal::StepperBaseCT<SCALAR, SCALAR>> stepperCost_;
    std::shared_ptr<internal::StepperBaseCT<SCALAR, state_vector>> stepperCostDX0_;
    std::shared_ptr<internal::StepperBaseCT<SCALAR, control_vector>> stepperCostDU0_;

    size_t costIndex_;
    size_t dX0Index_;
    size_t dU0Index_;

    std::function<void (const state_vector&, const SCALAR, state_vector&)> xDot_;

    std::shared_ptr<internal::StepperBaseCT<SCALAR, state_vector>> stepperState_;

};


}
}

#endif
