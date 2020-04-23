/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/rbd/physics/PinocchioRBD.h>

#include <ct/rbd/rbd.h>

#include "ExampleRobot.h"
#include "../ctRbdTestDir.h"

#include <gtest/gtest.h>


const bool verbose = false;
using namespace ct::rbd;


TEST(RBD_Pinocchio_Test, dynamics_test)
{
    const size_t state_dim = ExampleRobot::NJOINTS * 2;
    const size_t control_dim = ExampleRobot::NJOINTS;

    // create instance of pinocchio
    std::shared_ptr<PinocchioRBD<ExampleRobot>> robot(new PinocchioRBD<ExampleRobot>());

    // loading robot from file
    const std::string path = ct::rbd::CT_RBD_TEST_DIRECTORY + "/physics/ExampleRobot.urdf";
    ASSERT_TRUE(robot->loadModelFromFile(path, verbose));

    // test a controlled system
    std::shared_ptr<ct::rbd::FixBaseRobotSystem<ExampleRobot>> controlledSystem( new ct::rbd::FixBaseRobotSystem<ExampleRobot>(robot));
    ct::core::StateVector<state_dim> x = ct::core::StateVector<state_dim>::Random();
    ct::core::ControlVector<control_dim> u = ct::core::ControlVector<control_dim>::Random();
    double t = 0.0;
    ct::core::StateVector<state_dim> dxdt;
    controlledSystem->computeControlledDynamics(x, t, u, dxdt);

    ct::rbd::FixBaseRobotSystemLinear<ExampleRobot> linearSystem(robot);
    ct::core::StateMatrix<state_dim> A, A_numdiff;
    ct::core::StateControlMatrix<state_dim, control_dim> B, B_numdiff;
    linearSystem.getDerivatives(A, B, x, u, t);

    // create numdiff system linearizer to compare results
    ct::core::SystemLinearizer<state_dim, control_dim> systemLinearizer (controlledSystem);
    systemLinearizer.getDerivatives(A_numdiff, B_numdiff, x, u, t);
    ASSERT_TRUE(A_numdiff.isApprox(A, 1e-7));
    ASSERT_TRUE(B_numdiff.isApprox(B, 1e-7));

    ASSERT_TRUE(dxdt.isApprox(A * x + B * u)); // << -- why does this not hold?
}


template <typename ROB>
void compareTwoRbdInstances(std::shared_ptr<RigidBodyDynamics<ROB>> rbd1,
    std::shared_ptr<RigidBodyDynamics<ROB>> rbd2,
    size_t nTests,
    const typename ROB::Frames testFrame)
{
    using JointPosition_t = typename RigidBodyDynamics<ROB>::JointPosition_t;
    using JointVelocity_t = typename RigidBodyDynamics<ROB>::JointVelocity_t;
    using JointTorque_t = typename RigidBodyDynamics<ROB>::JointTorque_t;
    using JointAcceleration_t = typename RigidBodyDynamics<ROB>::JointAcceleration_t;
    using Twist_t = typename RigidBodyDynamics<ROB>::Twist_t;
    using Pose_t = typename RigidBodyDynamics<ROB>::Pose_t;

    for (size_t i = 0; i < nTests; i++)
    {
        JointPosition_t p = JointPosition_t::Random();
        JointVelocity_t v = JointVelocity_t::Random();

        // TEST -- dynamics tests
        JointTorque_t tau = JointTorque_t::Random();

        // compute forward dynamics for random values
        JointAcceleration_t accOut, accOut2;
        rbd1->computeForwardDynamics(p, v, tau, accOut);
        rbd2->computeForwardDynamics(p, v, tau, accOut2);
        ASSERT_TRUE(accOut.isApprox(accOut2));

        // compute inverse dynamics and assert that resulting torque is identical to original input torque
        JointTorque_t tau_rev, tauReverse2;
        rbd1->computeInverseDynamics(p, v, accOut, tau_rev);
        rbd2->computeInverseDynamics(p, v, accOut, tauReverse2);
        ASSERT_TRUE(tau_rev.isApprox(tauReverse2));

        // TEST 3 -- test gravity compensation
        p.setRandom();
        JointTorque_t tau_grav, tauGrav2;
        rbd1->computeGravityCompensation(p, tau_grav);
        rbd2->computeGravityCompensation(p, tauGrav2);
        ASSERT_TRUE(tau_grav.isApprox(tauGrav2));

        // TEST - kinematics tests
        Pose_t ee_pose_in_base_coordinates, base_pose_in_ee_coordinates;
        Pose_t ee_pose_in_base_coordinates_2, base_pose_in_ee_coordinates_2;

        rbd1->computeFramePoseInBaseCoordinates(p, testFrame, ee_pose_in_base_coordinates);
        rbd2->computeFramePoseInBaseCoordinates(p, testFrame, ee_pose_in_base_coordinates_2);
        ASSERT_TRUE(ee_pose_in_base_coordinates.isApprox(ee_pose_in_base_coordinates_2));

        rbd1->computeBasePoseInFrameCoordinates(p, testFrame, base_pose_in_ee_coordinates);
        rbd2->computeBasePoseInFrameCoordinates(p, testFrame, base_pose_in_ee_coordinates_2);
        ASSERT_TRUE(base_pose_in_ee_coordinates.isApprox(base_pose_in_ee_coordinates_2));

        Eigen::Matrix<double, 6, ROB::NJOINTS> jac, jac2;

        // check jacobian in local coordinates
        rbd1->computeSpatialJacobianInFrameCoordinates(p, testFrame, jac, true);
        rbd2->computeSpatialJacobianInFrameCoordinates(p, testFrame, jac2, true);
        ASSERT_TRUE(jac.isApprox(jac2));

        // check jacobian in base coordinates
        rbd1->computeSpatialJacobianInBaseCoordinates(p, testFrame, jac, true);
        rbd2->computeSpatialJacobianInBaseCoordinates(p, testFrame, jac2, true);
        ASSERT_TRUE(jac.isApprox(jac2));


        // TWIST TESTS ---------------------------------------------------------

        // use fresh values for twist tests
        p.setRandom();
        v.setRandom();

        // compare the twists as computed by the kinematics methods directly, assert that results from Jacobians are the same
        Twist_t rbd1_twist_base, rbd1_twist_local, rbd2_twist_local, rbd2_twist_base;
        Eigen::Matrix<double, 6, ROB::NJOINTS> jac_rbd1_local, jac_rbd1_base, jac_rbd2_local, jac_rbd2_base;

        rbd1->computeTwistInBaseCoordinates(p, v, testFrame, rbd1_twist_base, true);
        rbd1->computeTwistInFrameCoordinates(p, v, testFrame, rbd1_twist_local, true);
        rbd2->computeTwistInBaseCoordinates(p, v, testFrame, rbd2_twist_base, true);
        rbd2->computeTwistInFrameCoordinates(p, v, testFrame, rbd2_twist_local, true);

        rbd1->computeSpatialJacobianInFrameCoordinates(p, testFrame, jac_rbd1_local, true);
        rbd1->computeSpatialJacobianInBaseCoordinates(p, testFrame, jac_rbd1_base, true);
        rbd2->computeSpatialJacobianInFrameCoordinates(p, testFrame, jac_rbd2_local, true);
        rbd2->computeSpatialJacobianInBaseCoordinates(p, testFrame, jac_rbd2_base, true);

        // assert that twist is identical to respective "Jacobian * dq"
        ASSERT_TRUE(rbd1_twist_base.isApprox(jac_rbd1_base * v));
        ASSERT_TRUE(rbd1_twist_local.isApprox(jac_rbd1_local * v));
        ASSERT_TRUE(rbd2_twist_base.isApprox(jac_rbd2_base * v));
        ASSERT_TRUE(rbd2_twist_local.isApprox(jac_rbd2_local * v));

        ASSERT_TRUE(rbd1_twist_base.isApprox(rbd2_twist_base));
        ASSERT_TRUE(rbd1_twist_local.isApprox(rbd2_twist_local));
    }
}


TEST(RBD_Pinocchio_Test, EvaluateEngineTest)
{
    using JointPosition_t = PinocchioRBD<ExampleRobot>::JointPosition_t;
    using JointVelocity_t = PinocchioRBD<ExampleRobot>::JointVelocity_t;
    using JointTorque_t = PinocchioRBD<ExampleRobot>::JointTorque_t;
    using JointAcceleration_t = PinocchioRBD<ExampleRobot>::JointAcceleration_t;
    using Twist_t = PinocchioRBD<ExampleRobot>::Twist_t;
    using Pose_t = PinocchioRBD<ExampleRobot>::Pose_t;
    using Wrench_t = PinocchioRBD<ExampleRobot>::Wrench_t;

    // create instance of pinocchio
    PinocchioRBD<ExampleRobot> robot;

    // TEST 1 -- test loading robot from file
    const std::string path = CT_RBD_TEST_DIRECTORY + "/physics/ExampleRobot.urdf";
    ASSERT_TRUE(robot.loadModelFromFile(path, verbose));

    // Joint limits from URDF file, manually read (first joint is continous)
    JointPosition_t urdf_lower_lim, urdf_upper_lim;
    urdf_lower_lim << std::numeric_limits<double>::max(), -1.7628, -2.86, -3.03, -2.86, 0.04, -2.82;
    urdf_upper_lim << std::numeric_limits<double>::lowest(), 1.7628, 2.8973, -0.0698, 2.86, 3.71, 2.82;
    // TEST -- test joint limits
    JointPosition_t config_lower_lim, config_upper_lim;
    robot.getPositionLimits(config_lower_lim, config_upper_lim);
    ASSERT_TRUE(config_lower_lim.isApprox(urdf_lower_lim));
    ASSERT_TRUE(config_upper_lim.isApprox(urdf_upper_lim));

    // Effort limits from URDF file, manually read
    JointTorque_t urdf_effort_lim;
    urdf_effort_lim << 120, 120, 120, 120, 50, 50, 50;

    // TEST -- test effort limits
    JointTorque_t effort_lim;
    robot.getEffortLimits(effort_lim);
    ASSERT_TRUE(effort_lim.isApprox(urdf_effort_lim));

    // Velocity limits from URDF file, manually read
    JointVelocity_t urdf_velocity_lim;
    urdf_velocity_lim << 2.3925, 2.3925, 2.3925, 2.3925, 2.871, 2.871, 2.871;

    // TEST -- test velocity limits
    JointVelocity_t velocity_lim;
    robot.getVelocityLimits(velocity_lim);
    ASSERT_TRUE(velocity_lim.isApprox(urdf_velocity_lim));

    size_t nTests = 10;  // run this test several times

    for (size_t i = 0; i < nTests; i++)
    {
        JointPosition_t p = JointPosition_t::Random();
        JointVelocity_t v = JointVelocity_t::Random();

        // TEST -- dynamics tests
        JointTorque_t tau = JointTorque_t::Random();

        // compute forward dynamics for random values
        JointAcceleration_t accOut;
        robot.computeForwardDynamics(p, v, tau, accOut);

        // compute inverse dynamics and assert that resulting torque is identical to original input torque
        JointTorque_t tau_rev;
        robot.computeInverseDynamics(p, v, accOut, tau_rev);
        ASSERT_TRUE(tau.isApprox(tau_rev));

        // TEST 3 -- test inverse dynamics vs gravity compensation
        // create zero-v zero acc testcase where the inverse dynamics must be equal to gravity compensation
        p.setRandom();
        v.setZero();
        JointAcceleration_t acc_in = JointAcceleration_t::Zero();
        JointTorque_t tau_grav, tau_id;

        robot.computeGravityCompensation(p, tau_grav);
        robot.computeInverseDynamics(p, v, acc_in, tau_id);
        ASSERT_TRUE(tau_id.isApprox(tau_grav));


        // TEST - kinematics tests
        Pose_t ee_pose_in_base_coordinates, base_pose_in_ee_coordinates;
        robot.computeFramePoseInBaseCoordinates(p, ExampleRobot::TOOL, ee_pose_in_base_coordinates);
        robot.computeBasePoseInFrameCoordinates(p, ExampleRobot::TOOL, base_pose_in_ee_coordinates);

        // check that the distance base-endeffector is the same for both transformations
        ASSERT_NEAR(ee_pose_in_base_coordinates.position().toImplementation().norm(),
            base_pose_in_ee_coordinates.position().toImplementation().norm(), 1e-6);


        // express the ee-pose in the base frame, then the world-pose in the base frame, and ensure that their combination
        // results in the same ee-world-pose than the direct compuation.
        Pose_t world_pose_in_base_coordinates, ee_pose_in_world_coordinates, ee_pose_in_world_coordinates_ref;
        robot.computeWorldPoseInFrameCoordinates(p, ExampleRobot::BASE, world_pose_in_base_coordinates);
        ee_pose_in_world_coordinates = ee_pose_in_base_coordinates.inReferenceFrame(world_pose_in_base_coordinates);
        robot.computeFramePoseInWorldCoordinates(p, ExampleRobot::TOOL, ee_pose_in_world_coordinates_ref);

        // these should be identical
        ASSERT_TRUE(ee_pose_in_world_coordinates.isNear(ee_pose_in_world_coordinates_ref));


        // check if jacobian calculations deliver bounded numeric values
        Eigen::Matrix<double, 6, ExampleRobot::NJOINTS> jac;

        // check jacobian in world coordinates
        jac.setConstant(std::numeric_limits<double>::quiet_NaN());  // initialize as NaN
        robot.computeSpatialJacobianInWorldCoordinates(p, ExampleRobot::TOOL, jac);
        ASSERT_FALSE(jac.hasNaN());

        // check jacobian in local coordinates
        jac.setConstant(std::numeric_limits<double>::quiet_NaN());  // initialize as NaN
        robot.computeSpatialJacobianInFrameCoordinates(p, ExampleRobot::TOOL, jac);
        ASSERT_FALSE(jac.hasNaN());

        // check jacobian in base coordinates
        jac.setConstant(std::numeric_limits<double>::quiet_NaN());  // initialize as NaN
        robot.computeSpatialJacobianInBaseCoordinates(p, ExampleRobot::TOOL, jac);
        ASSERT_FALSE(jac.hasNaN());

        // check twist
        Twist_t pin_twist_world;
        v.setRandom();
        robot.computeTwistInWorldCoordinates(p, v, ExampleRobot::TOOL, pin_twist_world);
        robot.computeSpatialJacobianInWorldCoordinates(p, ExampleRobot::TOOL, jac);
        ASSERT_TRUE(pin_twist_world.isApprox(jac * v));


        // check inverse dynamics with external forces
        p.setRandom();
        v.setRandom();
        tau.setRandom();
        // create random external wrench in tool frame...
        Wrench_t f_ext_in_tool_frame = Wrench_t::Random();
        // ...and transform it to ROB base frame for reference
        Wrench_t f_ext_in_base_frame =
            robot.fromFrameToBaseCoordinates(f_ext_in_tool_frame, p, ExampleRobot::TOOL, true);
        // get TCP jacobian and run forward dynamics including the external wrench
        robot.computeSpatialJacobianInBaseCoordinates(p, ExampleRobot::TOOL, jac);
        robot.computeForwardDynamics(p, v, tau + jac.transpose() * f_ext_in_base_frame, accOut);
        // use the resulting acceleration "accOut" in the inverse dynamics with ext-forces
        JointTorque_t idTorque;
        robot.computeInverseDynamics(p, v, accOut, f_ext_in_tool_frame, idTorque);
        // the expected result is that the computed ID joint torque should match the original random joint torque (without ext forces)
        ASSERT_TRUE(idTorque.isApprox(tau));
    }
}


/*
 * @brief clone instances instance and check results for equality
 */
TEST(RBD_Pinocchio_Test, CloneInstancesTest)
{
    size_t nTests = 2;

    // create instance of pinocchio
    std::shared_ptr<PinocchioRBD<ExampleRobot>> robot(new PinocchioRBD<ExampleRobot>());

    // loading robot from file
    const std::string path = ct::rbd::CT_RBD_TEST_DIRECTORY + "/physics/ExampleRobot.urdf";
    ASSERT_TRUE(robot->loadModelFromFile(path, verbose));

    // create a clone of pinocchio and compare
    std::shared_ptr<PinocchioRBD<ExampleRobot>> clonedInstance(robot->clone());
    ASSERT_TRUE(robot.unique());
    ASSERT_TRUE(clonedInstance.unique());
    compareTwoRbdInstances<ExampleRobot>(robot, clonedInstance, nTests, ExampleRobot::TOOL);

    // delete the original pinocchio instance
    robot.reset();

    // create a second clone and compare both clones
    std::shared_ptr<PinocchioRBD<ExampleRobot>> clonedInstance2(clonedInstance->clone());
    ASSERT_TRUE(clonedInstance2.unique());
    ASSERT_TRUE(clonedInstance.unique());
    compareTwoRbdInstances<ExampleRobot>(clonedInstance, clonedInstance2, nTests, ExampleRobot::TOOL);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
