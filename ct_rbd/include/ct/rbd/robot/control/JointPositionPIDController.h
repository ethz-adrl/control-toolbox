/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

#include <vector>

namespace ct {
namespace rbd {

/**
 * \brief A joint position controller using a PID controller for all joints
 *
 * \tparam NJOINTS number of joints of the robot
 */
template <size_t NJOINTS>
class JointPositionPIDController : public ct::core::Controller<2 * NJOINTS, NJOINTS>
{
public:
    static const size_t STATE_DIM = 2 * NJOINTS;
    static const size_t CONTROL_DIM = NJOINTS;

    typedef std::shared_ptr<JointPositionPIDController<NJOINTS>> Ptr;
    typedef ct::core::PIDController<double> PIDController;

    virtual JointPositionPIDController<NJOINTS>* clone() const override;

    JointPositionPIDController(
        const Eigen::Matrix<double, NJOINTS, 1>& desiredPosition = Eigen::Matrix<double, NJOINTS, 1>::Zero(),
        const Eigen::Matrix<double, NJOINTS, 1>& desiredVelocity = Eigen::Matrix<double, NJOINTS, 1>::Zero(),
        const std::vector<PIDController::parameters_t>& parameters = std::vector<PIDController::parameters_t>(NJOINTS,
            PIDController::parameters_t()));

    JointPositionPIDController(const Eigen::Matrix<double, NJOINTS, 1>& desiredPosition,
        const Eigen::Matrix<double, NJOINTS, 1>& desiredVelocity,
        const PIDController::parameters_t& parameters);

    virtual ~JointPositionPIDController();

    void computeControl(const core::StateVector<STATE_DIM>& state,
        const core::Time& t,
        core::ControlVector<NJOINTS>& control) override;

    void setDesiredPosition(const Eigen::Matrix<double, NJOINTS, 1>& desiredPosition);

    void setDesiredPosition(double desiredPosition, int jointId);

    void setAllPIDGains(double kp, double ki, double kd);

    void reset();

protected:
    std::vector<PIDController> jointControllers_;
};

}  // namespace rbd
}  // namespace ct
