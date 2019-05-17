/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {

template <size_t NJOINTS>
JointPositionPIDController<NJOINTS>* JointPositionPIDController<NJOINTS>::clone() const
{
    throw std::runtime_error("RBD: JointPositionPIDController.h, clone() not implemented");
}

template <size_t NJOINTS>
JointPositionPIDController<NJOINTS>::~JointPositionPIDController()
{
}

template <size_t NJOINTS>
JointPositionPIDController<NJOINTS>::JointPositionPIDController(
    const Eigen::Matrix<double, NJOINTS, 1>& desiredPosition,
    const Eigen::Matrix<double, NJOINTS, 1>& desiredVelocity,
    const std::vector<PIDController::parameters_t>& parameters)
{
    assert(parameters.size() == NJOINTS);

    for (size_t i = 0; i < NJOINTS; i++)
    {
        jointControllers_.push_back(
            PIDController(parameters[i], PIDController::setpoint_t(desiredPosition(i), desiredVelocity(i))));
    }
}

template <size_t NJOINTS>
JointPositionPIDController<NJOINTS>::JointPositionPIDController(
    const Eigen::Matrix<double, NJOINTS, 1>& desiredPosition,
    const Eigen::Matrix<double, NJOINTS, 1>& desiredVelocity,
    const PIDController::parameters_t& parameters)
{
    for (size_t i = 0; i < NJOINTS; i++)
    {
        jointControllers_.push_back(
            PIDController(parameters, PIDController::setpoint_t(desiredPosition(i), desiredVelocity(i))));
    }
}

template <size_t NJOINTS>
void JointPositionPIDController<NJOINTS>::computeControl(const core::StateVector<STATE_DIM>& state,
    const core::Time& t,
    core::ControlVector<NJOINTS>& control)
{
    ct::rbd::JointState<NJOINTS> jstate(state);

    for (size_t i = 0; i < NJOINTS; i++)
    {
        control(i) = jointControllers_[i].computeControl(jstate.getPosition(i), jstate.getVelocity(i), t);
    }
}

template <size_t NJOINTS>
void JointPositionPIDController<NJOINTS>::setDesiredPosition(const Eigen::Matrix<double, NJOINTS, 1>& desiredPosition)
{
    for (size_t i = 0; i < NJOINTS; i++)
    {
        jointControllers_[i].setDesiredState(desiredPosition(i));
    }
}

template <size_t NJOINTS>
void JointPositionPIDController<NJOINTS>::setDesiredPosition(double desiredPosition, int jointId)
{
    assert(0 <= jointId && jointId < NJOINTS);  // assuming first joint has index 0

    jointControllers_[jointId].setDesiredState(desiredPosition);
}

template <size_t NJOINTS>
void JointPositionPIDController<NJOINTS>::setAllPIDGains(double kp, double ki, double kd)
{
    PIDController::parameters_t parameters;
    parameters.k_p = kp;
    parameters.k_i = ki;
    parameters.k_d = kd;

    for (size_t i = 0; i < NJOINTS; i++)
    {
        jointControllers_[i].changeParameters(parameters);
    }
}

template <size_t NJOINTS>
void JointPositionPIDController<NJOINTS>::reset()
{
    for (size_t i = 0; i < NJOINTS; i++)
    {
        jointControllers_[i].reset();
    }
}


}  // namespace rbd
}  // namespace ct
