/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "ActuatorDynamics.h"

namespace ct {
namespace rbd {

/*!
 * Series-elastic actuator dynamics modeled as a series of motor, gearbox and a spring.
 *
 * Control Input: Motor Velocity
 * Actuator State: Gear Position
 * Actuator State Derivative: Gear Velocity
 *
 * \note The advantage of choosing the the gear position as state is that no calibration on the motor position is required.
 * In a SEA, the gear position is typically known anyway.
 *
 */
template <size_t NJOINTS, typename SCALAR = double>
class SEADynamicsFirstOrder : public ActuatorDynamics<NJOINTS, NJOINTS, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ActuatorDynamics<NJOINTS, NJOINTS, SCALAR> BASE;

    //! constructor assuming unit amplification
    SEADynamicsFirstOrder(double k_spring, double gear_ratio);

    //! destructor
    virtual ~SEADynamicsFirstOrder();

    //! deep cloning
    virtual SEADynamicsFirstOrder<NJOINTS, SCALAR>* clone() const override;

    virtual void computeActuatorDynamics(const JointState<NJOINTS, SCALAR>& robotJointState,
        const ct::core::StateVector<NJOINTS, SCALAR>& state,
        const SCALAR& t,
        const ct::core::ControlVector<NJOINTS, SCALAR>& control,
        ct::core::StateVector<NJOINTS, SCALAR>& derivative) override;

    virtual core::ControlVector<NJOINTS, SCALAR> computeControlOutput(
        const JointState<NJOINTS, SCALAR>& robotJointState,
        const typename BASE::act_state_vector_t& actState) override;

    virtual ct::core::StateVector<NJOINTS, SCALAR> computeStateFromOutput(
        const JointState<NJOINTS, SCALAR>& refRobotJointState,
        const core::ControlVector<NJOINTS, SCALAR>& refControl) override;

private:
    SCALAR k_;  //! spring constant
    SCALAR r_;  //! gear ratio, defined in terms of input/output
};


}  // namespace rbd
}  // namespace ct
