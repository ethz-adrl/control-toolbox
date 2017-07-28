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


#ifndef CT_CORE_INTERNAL_STEPPERS_CT_H_
#define CT_CORE_INTERNAL_STEPPERS_CT_H_

#include <ct/core/systems/System.h>

namespace ct {
namespace core {
namespace internal {

template <typename SCALAR, typename MatrixType>
class StepperBaseCT
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void do_step(
        const std::function<void (const MatrixType&, SCALAR, MatrixType&)>& rhs,
        MatrixType& stateInOut,
        const SCALAR time,
        const SCALAR dt
        ) = 0;

protected:
    
};


template<typename SCALAR, typename MatrixType>
class StepperEulerCT : public StepperBaseCT<SCALAR, MatrixType>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StepperEulerCT(){}

    virtual void do_step(
        const std::function<void (const MatrixType&, SCALAR, MatrixType&)>& rhs,
        MatrixType& stateInOut,
        const SCALAR time,
        const SCALAR dt
        ) override
    {
        rhs(stateInOut, time, derivative_);
        stateInOut += dt * derivative_;
    }

private:
    MatrixType derivative_;

};

template<typename SCALAR, typename MatrixType>
class StepperRK4CT : public StepperBaseCT<SCALAR, MatrixType>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StepperRK4CT() 
    :
    oneSixth_(SCALAR(1.0 / 6.0))
    {

    }

    virtual void do_step(
        const std::function<void (const MatrixType&, SCALAR, MatrixType&)>& rhs,
        MatrixType& stateInOut,
        const SCALAR time,
        const SCALAR dt
        ) override
    {
        SCALAR halfStep = SCALAR(0.5) * dt;
        SCALAR timePlusHalfStep = time + halfStep;
        rhs(stateInOut, time, k1_);
        rhs(stateInOut + halfStep * k1_, timePlusHalfStep, k2_);
        rhs(stateInOut + halfStep * k2_, timePlusHalfStep, k3_);
        rhs(stateInOut + dt * k3_, time + dt, k4_);
        stateInOut += oneSixth_ * dt * (k1_ + SCALAR(2.0) * k2_ + SCALAR(2.0) * k3_ + k4_);
    }

private:
    MatrixType k1_;
    MatrixType k2_;
    MatrixType k3_;
    MatrixType k4_;
    SCALAR oneSixth_;

};


}
}
}



#endif