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

#pragma once

#include "ActuatorDynamics.h"

namespace ct {
namespace rbd {

/*!
 * Actuator Dynamics modelled as second order system, an oscillator with damping.
 */
template <size_t NJOINTS, typename SCALAR = double>
class SecondOrderActuatorDynamics : public ActuatorDynamics<NJOINTS, 2 * NJOINTS, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ActuatorDynamics<NJOINTS, 2 * NJOINTS, SCALAR> BASE;

    //! constructor
    SecondOrderActuatorDynamics(SCALAR w_n, SCALAR zeta = SCALAR(1.0), SCALAR g_dc = SCALAR(1.0));

    //! destructor
    virtual ~SecondOrderActuatorDynamics();

    //! deep cloning
    virtual SecondOrderActuatorDynamics<NJOINTS, SCALAR>* clone() const override;


    virtual void computePdot(const typename BASE::act_state_vector_t& x,
        const typename BASE::act_vel_vector_t& v,
        const ct::core::ControlVector<NJOINTS, SCALAR>& control,
        typename BASE::act_pos_vector_t& pDot) override;


    virtual void computeVdot(const typename BASE::act_state_vector_t& x,
        const typename BASE::act_pos_vector_t& p,
        const ct::core::ControlVector<NJOINTS, SCALAR>& control,
        typename BASE::act_vel_vector_t& vDot) override;


    virtual core::ControlVector<NJOINTS, SCALAR> computeControlOutput(
        const ct::rbd::tpl::JointState<NJOINTS, SCALAR>& robotJointState,
        const typename BASE::act_state_vector_t& actState) override;


private:
    ct::core::SecondOrderSystem oscillator_;
};
}
}
