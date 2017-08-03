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

#ifndef INCLUDE_CT_CORE_INTEGRATION_INTERNAL_STEPPERBASE_H_
#define INCLUDE_CT_CORE_INTEGRATION_INTERNAL_STEPPERBASE_H_

namespace ct {
namespace core {
namespace internal {

template <typename MatrixType, typename SCALAR = double>
class StepperBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StepperBase(){}

    virtual ~StepperBase(){}

    virtual void integrate_n_steps(
            const std::function<void (const MatrixType&, MatrixType&, SCALAR)>& rhs,
            MatrixType& state,
            const SCALAR& startTime,
            size_t numSteps,
            SCALAR dt)
    {
        throw std::runtime_error("Integrate_n_steps not implemented for the stepper type");
    }

    virtual void integrate_n_steps(
            std::function<void (const MatrixType& x, const SCALAR& t)> observer,
            const std::function<void (const MatrixType&, MatrixType&, SCALAR)>& rhs,
            MatrixType& state,
            const SCALAR& startTime,
            size_t numSteps,
            SCALAR dt)
    {
        throw std::runtime_error("Integrate_n_steps not implemented for the stepper type");
    }


    virtual void integrate_const(
            const std::function<void (const MatrixType&, MatrixType&, SCALAR)>& rhs,
            MatrixType& state,
            const SCALAR& startTime,
            const SCALAR& finalTime,
            SCALAR dt)
    {
        throw std::runtime_error("integrate_const not implemented for the stepper type");
    }

    virtual void integrate_const(
            std::function<void (const MatrixType& x, const SCALAR& t)> observer,
            const std::function<void (const MatrixType&, MatrixType&, SCALAR)>& rhs,
            MatrixType& state,
            const SCALAR& startTime,
            const SCALAR& finalTime,
            SCALAR dt)
    {
        throw std::runtime_error("integrate_const not implemented for the stepper type");
    }

    virtual void integrate_adaptive(
            const std::function<void (const MatrixType&, MatrixType&, SCALAR)>& rhs,
            MatrixType& state,
            const SCALAR& startTime,
            const SCALAR& finalTime,
            SCALAR dtInitial = SCALAR(0.01))
    {
        throw std::runtime_error("integrate_adaptive not implemented for the stepper type");
    }

    virtual void integrate_adaptive(
            std::function<void (const MatrixType& x, const SCALAR& t)> observer,
            const std::function<void (const MatrixType&, MatrixType&, SCALAR)>& rhs,
            MatrixType& state,
            const SCALAR& startTime,
            const SCALAR& finalTime,
            const SCALAR dtInitial = SCALAR(0.01))
    {
        throw std::runtime_error("integrate_adaptive not implemented for the stepper type");        
    }

    virtual void integrate_times(
            std::function<void (const MatrixType& x, const SCALAR& t)> observer,
            const std::function<void (const MatrixType&, MatrixType&, SCALAR)>& rhs,
            MatrixType& state,
            const tpl::TimeArray<SCALAR>& timeTrajectory,
            SCALAR dtInitial = SCALAR(0.01))
    {
        throw std::runtime_error("integrate_times not implemented for the stepper type");
    }

protected:


    
};



}
}
}


#endif