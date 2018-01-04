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
 * Series-elastic actuator dynamics modelled as a spring, control input is the motor velocity
 */
template <size_t NJOINTS, typename SCALAR = double>
class SEADynamicsFirstOrder : public ActuatorDynamics<NJOINTS, NJOINTS, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ActuatorDynamics<NJOINTS, NJOINTS, SCALAR> BASE;

    //! constructor assuming unit amplification
    SEADynamicsFirstOrder(double k_spring);

    //! destructor
    virtual ~SEADynamicsFirstOrder();

    //! deep cloning
    virtual SEADynamicsFirstOrder<NJOINTS, SCALAR>* clone() const override;

    virtual void computeActuatorDynamics(const ct::rbd::tpl::JointState<NJOINTS, SCALAR>& robotJointState,
        const ct::core::StateVector<NJOINTS, SCALAR>& state,
        const SCALAR& t,
        const ct::core::ControlVector<NJOINTS, SCALAR>& control,
        ct::core::StateVector<NJOINTS, SCALAR>& derivative) override;

    virtual core::ControlVector<NJOINTS, SCALAR> computeControlOutput(
        const ct::rbd::tpl::JointState<NJOINTS, SCALAR>& robotJointState,
        const typename BASE::act_state_vector_t& actState) override;

    virtual ct::core::StateVector<NJOINTS, SCALAR> computeStateFromOutput(
        const ct::rbd::tpl::JointState<NJOINTS, SCALAR>& refRobotJointState,
        const core::ControlVector<NJOINTS, SCALAR>& refControl) override;

private:
    SCALAR k_;  //! spring constant
};


}  // namespace rbd
}  // namespace ct
