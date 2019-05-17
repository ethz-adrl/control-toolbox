/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/
/*!
 * \brief This is a basic example showing how to use the simple Arm Trajectory Generator
 */


#include <ct/rbd/rbd.h>
#include "plotResultsTrajectories.h"

int main(int argc, char** argv)
{
    const size_t njoints = 6;  // 6 DoF robot arm

    // variables for start and end position, maximal velocity and maximal acceleration
    ct::rbd::JointState<njoints>::Position start_joints, end_joints;
    ct::rbd::JointState<njoints>::Velocity max_vel;
    ct::rbd::JointAcceleration<njoints>::Acceleration max_acc;

    // initialize with arbitrary values
    start_joints << 1.0, 2.0, -0.2, -0.033333, -M_PI / 3, -0.1;
    end_joints << 2.0, -0.2, -0.033333, -M_PI / 3, 0.0, 0.1;
    max_vel << 0.18, 0.22, 0.4, 0.3, M_PI / 10, 0.3;
    max_acc << 0.2, 0.2, M_PI / 6, M_PI / 7, 0.2, 0.2;

    // create and init an instance of the joint trajectory generator
    ct::rbd::SimpleArmTrajectoryGenerator<njoints> traj_generator(max_vel, max_acc);

    bool verbose = true;
    double dt = 0.02;
    ct::core::StateTrajectory<2 * njoints> testTraj =
        traj_generator.generateTrajectory(start_joints, end_joints, dt, verbose);

    plotResultsTrajectories<njoints>(testTraj);
    return 1;
}
