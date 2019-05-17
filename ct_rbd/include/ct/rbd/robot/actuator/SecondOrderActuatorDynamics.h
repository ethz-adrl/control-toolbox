/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "ActuatorDynamics.h"

namespace ct {
namespace rbd {

/*!
 * Actuator Dynamics modeled as second order system, an oscillator with damping.
 *
 * \warning This is wrong - actually the simple oscillator is not a symplectic system (if damping != 0)
 */
template <size_t NJOINTS, typename SCALAR = double>
class SecondOrderActuatorDynamics : public ActuatorDynamics<2 * NJOINTS, NJOINTS, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ActuatorDynamics<2 * NJOINTS, NJOINTS, SCALAR> BASE;

    //! constructor assuming unit amplification
    SecondOrderActuatorDynamics(double w_n, double zeta);

    //! constructor assuming custom amplification, set g_dc = w_n*w_n
    SecondOrderActuatorDynamics(double w_n, double zeta, double g_dc);

    //! constructor assuming unit amplification
    SecondOrderActuatorDynamics(std::vector<double> w_n, std::vector<double> zeta);

    //! constructor assuming custom amplification, set g_dc = w_n*w_n
    SecondOrderActuatorDynamics(std::vector<double> w_n, std::vector<double> zeta, std::vector<double> g_dc);

    //! destructor
    virtual ~SecondOrderActuatorDynamics();

    //! deep cloning
    virtual SecondOrderActuatorDynamics<NJOINTS, SCALAR>* clone() const override;

    virtual void computeActuatorDynamics(const JointState<NJOINTS, SCALAR>& robotJointState,
        const ct::core::StateVector<2 * NJOINTS, SCALAR>& state,
        const SCALAR& t,
        const ct::core::ControlVector<NJOINTS, SCALAR>& control,
        ct::core::StateVector<2 * NJOINTS, SCALAR>& derivative) override;

    virtual core::ControlVector<NJOINTS, SCALAR> computeControlOutput(
        const JointState<NJOINTS, SCALAR>& robotJointState,
        const typename BASE::act_state_vector_t& actState) override;

    virtual ct::core::StateVector<2 * NJOINTS, SCALAR> computeStateFromOutput(
        const JointState<NJOINTS, SCALAR>& refRobotJointState,
        const core::ControlVector<NJOINTS, SCALAR>& refControl) override;

private:
    std::vector<ct::core::tpl::SecondOrderSystem<SCALAR>> oscillators_;
};

}  // namespace rbd
}  // namespace ct
