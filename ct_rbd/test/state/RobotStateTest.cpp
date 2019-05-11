/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#include <ct/rbd/rbd.h>
#include <gtest/gtest.h>


template <typename T>
bool areEqual(const T v1, const T v2)
{
    return ((v1 - v2).array().abs().maxCoeff() == 0.0);
}


TEST(RobotStateTest, FloatingBaseInstantiationTest)
{
    const size_t njoints = 6;
    const size_t act_state_dim = 6;

    ct::rbd::FloatingBaseRobotState<njoints> robotStateTest1;
    ct::rbd::FloatingBaseRobotState<njoints, act_state_dim> robotStateTest2;
}


TEST(RobotStateTest, FixBaseInstantiationTest)
{
    const size_t njoints = 6;
    const size_t act_state_dim = 6;

    ct::rbd::FixBaseRobotState<njoints> robotStateTest1;
    ct::rbd::FixBaseRobotState<njoints, act_state_dim> robotStateTest2;
}


TEST(RobotStateTest, FloatingBaseStateComparison)
{
    const size_t njoints = 6;
    const size_t act_state_dim = 6;

    size_t nTests = 20;
    for (size_t i = 0; i < nTests; i++)
    {
        ct::rbd::RBDState<njoints> rbdState;
        rbdState.setRandom();

        ct::core::StateVector<act_state_dim> actuatorState;
        actuatorState.setRandom();

        ct::rbd::FloatingBaseRobotState<njoints, act_state_dim> robotState(rbdState, actuatorState);

        // test that the original RBDState and the one retrieved from RobotState are identical
        ASSERT_TRUE(rbdState.isApprox(robotState.rbdState()));

        // test that the original actuator state vector and the one retrieved from RobotState are identical
        ASSERT_TRUE(areEqual(actuatorState, robotState.actuatorState()));

        // test copy constructor
        ct::rbd::FloatingBaseRobotState<njoints, act_state_dim> robotStateCopied(robotState);
        ASSERT_TRUE(robotStateCopied.rbdState().isApprox(robotState.rbdState()));
        ASSERT_TRUE(areEqual(actuatorState, robotStateCopied.actuatorState()));

        // comparing basic accessor methods
        ASSERT_TRUE(areEqual(rbdState.jointPositions(), robotState.jointPositions()));
        ASSERT_TRUE(areEqual(rbdState.jointVelocities(), robotState.jointVelocities()));
        ASSERT_TRUE(areEqual(
            rbdState.baseLinearVelocity().toImplementation(), robotState.baseLinearVelocity().toImplementation()));
        ASSERT_TRUE(areEqual(rbdState.baseLocalAngularVelocity().toImplementation(),
            robotState.baseLocalAngularVelocity().toImplementation()));
        ASSERT_TRUE(areEqual(rbdState.baseVelocities().getVector(), robotState.baseVelocities().getVector()));

        // a series of state conversions and comparisons
        {
            // check robot state to full state with quaternion conversion
            ct::rbd::FloatingBaseRobotState<njoints, act_state_dim>::state_vector_quat_t temp =
                robotState.toStateVectorQuaternion();
            const size_t nstate_quat = ct::rbd::RBDState<njoints>::NSTATE_QUAT;
            // reconstructed states ...
            ct::core::StateVector<nstate_quat> state_quat_reconstr = temp.head<nstate_quat>();
            ct::core::StateVector<act_state_dim> actStateReconstr = temp.tail<act_state_dim>();
            // .. must be equal to original states
            ASSERT_TRUE(areEqual(state_quat_reconstr, rbdState.toStateVectorQuaternion()));
            ASSERT_TRUE(areEqual(actStateReconstr, actuatorState));
        }
        {
            // check robot state to full state with EulerXyzUnique() conversion
            ct::rbd::FloatingBaseRobotState<njoints, act_state_dim>::state_vector_euler_t temp =
                robotState.toStateVectorEulerXyzUnique();
            const size_t nstate_euler = ct::rbd::RBDState<njoints>::NSTATE;
            // reconstructed states ...
            ct::core::StateVector<nstate_euler> state_euler_reconstr = temp.head<nstate_euler>();
            ct::core::StateVector<act_state_dim> actStateReconstr = temp.tail<act_state_dim>();
            // .. must be equal to original states
            ASSERT_TRUE(areEqual(actStateReconstr, actuatorState));
            ASSERT_TRUE(areEqual(state_euler_reconstr, rbdState.toStateVectorEulerXyzUnique()));
        }
        {
            // check robot state to full state with EulerXyz() conversion
            ct::rbd::FloatingBaseRobotState<njoints, act_state_dim>::state_vector_euler_t temp =
                robotState.toStateVectorEulerXyz();
            const size_t nstate_euler = ct::rbd::RBDState<njoints>::NSTATE;
            // reconstructed states ...
            ct::core::StateVector<nstate_euler> state_euler_reconstr = temp.head<nstate_euler>();
            ct::core::StateVector<act_state_dim> actStateReconstr = temp.tail<act_state_dim>();
            // .. must be equal to original states
            ASSERT_TRUE(areEqual(actStateReconstr, actuatorState));
            ASSERT_TRUE(areEqual(state_euler_reconstr, rbdState.toStateVectorEulerXyz()));
        }
        {
            // random Euler State to robot state conversion (and back)
            ct::rbd::FloatingBaseRobotState<njoints, act_state_dim>::state_vector_euler_t temp;
            temp.setRandom();
            ct::rbd::FloatingBaseRobotState<njoints, act_state_dim> testRobotState;
            testRobotState.fromStateVectorEulerXyz(temp);
            ASSERT_TRUE(areEqual(testRobotState.toStateVectorEulerXyz(), temp));
        }
        {
            // random quaternion state to robot state conversion (and back)
            ct::rbd::FloatingBaseRobotState<njoints, act_state_dim>::state_vector_quat_t temp;
            temp.setRandom();

            // need to generate a consistent unit random quaternion (compatible with older Eigen versions)
            // and overwrite the previous random state with it
            Eigen::Vector3d euler;
            euler.setRandom();
            Eigen::Quaterniond rand_quat = Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitX()) *
                                           Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
                                           Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitZ());
            temp(0) = rand_quat.w();
            temp(1) = rand_quat.x();
            temp(2) = rand_quat.y();
            temp(3) = rand_quat.z();

            ct::rbd::FloatingBaseRobotState<njoints, act_state_dim> testRobotState(
                ct::rbd::RigidBodyPose::STORAGE_TYPE::QUAT);  // selecting the correct storage type is imporant here
            testRobotState.fromStateVectorQuaternion(temp);
            ASSERT_TRUE(areEqual(testRobotState.toStateVectorQuaternion(), temp));
        }
    }  //end for
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
