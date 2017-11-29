/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

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
