/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "../state/JointState.h"
#include "../state/JointAcceleration.h"

#include "SingleDOFTrajectoryGenerator.h"

namespace ct {
namespace rbd {

/*!
 * Generates basic joint trajectories according to the principle explained in SingleDOFTrajectoryGenerator.h.
 * Not all joints will reach their target at the same time, every joint drives up to its individual max velocity
 * and finishes when arrived.
 * Whole trajectory ends when last joint arrives at it's desired position.
 * */
template <size_t NJOINTS, typename SCALAR = double>
class SimpleArmTrajectoryGenerator
{
public:
    using Position = typename ct::rbd::JointState<NJOINTS, SCALAR>::Position;
    using Velocity = typename ct::rbd::JointState<NJOINTS, SCALAR>::Velocity;
    using Acceleration = typename ct::rbd::JointAcceleration<NJOINTS, SCALAR>::Acceleration;


    SimpleArmTrajectoryGenerator(const Velocity& kAbsMaxJointVelocities, const Acceleration& kAbsMaxJointAccelerations)
        : abs_max_joint_velocities_(kAbsMaxJointVelocities), abs_max_joint_accelerations_(kAbsMaxJointAccelerations)
    {
        // Initialize joint angle generators
        for (size_t i = 0; i < NJOINTS; i++)
        {
            joint_traj_generators_.push_back(SingleDOFTrajectoryGenerator());
        }
    }

    /*!
	 * Generate a Trajectory with a default sampling time of 20 ms
	 * -> sampling in constant time intervals
	 */
    core::StateTrajectory<2 * NJOINTS> generateTrajectory(const Position& start,
        const Position& end,
        const SCALAR sampling_time = SCALAR(0.02),
        bool verbose = false)
    {
        // Step 1 set bounds and determine the joint that takes the most time to execute and the
        // according time
        std::vector<SCALAR> planned_joint_times(NJOINTS);

        SCALAR x_new, v_new;

        for (size_t i = 0; i < NJOINTS; i++)
        {
            planned_joint_times[i] = joint_traj_generators_[i].setup(
                start(i), end(i), abs_max_joint_velocities_(i), abs_max_joint_accelerations_(i));
        }

        SCALAR time_horizon = *std::max_element(planned_joint_times.begin(), planned_joint_times.end());
        size_t num_traj_points = round(time_horizon / sampling_time) + 1;

        if (verbose)
        {
            std::cout << "SimpleArmTrajectoryGenerator: Time horizon of new Trajectory: " << time_horizon << " sec."
                      << std::endl;
            std::cout << "SimpleArmTrajectoryGenerator: Number of Sampling Points: 	   " << num_traj_points
                      << std::endl;
        }

        core::StateTrajectory<2 * NJOINTS, SCALAR> new_traj;

        for (size_t n = 0; n < num_traj_points; n++)
        {
            core::StateVector<2 * NJOINTS, SCALAR> new_state;
            double time = n * sampling_time;

            for (size_t i = 0; i < NJOINTS; i++)
            {
                joint_traj_generators_[i].queryTrajectory(time, x_new, v_new);
                new_state(i) = x_new;
                new_state(i + NJOINTS) = v_new;
            }

            new_traj.push_back(new_state, time, true);
        }


        return new_traj;
    }

private:
    std::vector<tpl::SingleDOFTrajectoryGenerator<SCALAR>> joint_traj_generators_;
    Velocity abs_max_joint_velocities_;
    Acceleration abs_max_joint_accelerations_;
};

}  // rbd
}  // ct
