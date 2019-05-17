/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/rbd/rbd.h>
#include "ct/rbd/state/RigidBodyPose.h"

#include <memory>
#include <array>

#include <gtest/gtest.h>


using namespace ct::rbd;

void testEqual(const RigidBodyPose& pose1, const RigidBodyPose& pose2)
{
    ASSERT_TRUE(pose1.isNear(pose2));
    ASSERT_TRUE(pose2.isNear(pose1));

    ASSERT_TRUE(pose2.getEulerAnglesXyz().isNear(pose1.getEulerAnglesXyz(), 1e-10));
    ASSERT_TRUE(pose2.getRotationQuaternion().isNear(pose1.getRotationQuaternion(), 1e-10));

    ASSERT_TRUE(pose2.getRotationMatrix().isNear(pose1.getRotationMatrix(), 1e-10));

    ASSERT_TRUE(pose2.position().toImplementation().isApprox(pose1.position().toImplementation(), 1e-10));

    ASSERT_TRUE(pose2.computeGravityB().isApprox(pose1.computeGravityB(), 1e-10));
    ASSERT_TRUE(pose2.computeGravityB6D().isApprox(pose1.computeGravityB6D(), 1e-10));
}

void testNotEqual(const RigidBodyPose& pose1, const RigidBodyPose& pose2)
{
    ASSERT_FALSE(pose1.isNear(pose2));
    ASSERT_FALSE(pose2.isNear(pose1));

    ASSERT_FALSE(pose2.getEulerAnglesXyz().isNear(pose1.getEulerAnglesXyz(), 1e-10));
    ASSERT_FALSE(pose2.getRotationQuaternion().isNear(pose1.getRotationQuaternion(), 1e-10));

    ASSERT_FALSE(pose2.getRotationMatrix().isNear(pose1.getRotationMatrix(), 1e-10));

    ASSERT_FALSE(pose2.computeGravityB().isApprox(pose1.computeGravityB(), 1e-10));
    ASSERT_FALSE(pose2.computeGravityB6D().isApprox(pose1.computeGravityB6D(), 1e-10));
}

TEST(RigidBodyPoseTest, storageTest)
{
    RigidBodyPose poseQuat(RigidBodyPose::QUAT);
    RigidBodyPose poseEuler(RigidBodyPose::EULER);

    ASSERT_EQ(poseQuat.getStorageType(), RigidBodyPose::QUAT);
    ASSERT_EQ(poseEuler.getStorageType(), RigidBodyPose::EULER);
}

TEST(RigidBodyPoseTest, asignmentTest)
{
    RigidBodyPose poseQuat(RigidBodyPose::QUAT);
    RigidBodyPose poseEuler(RigidBodyPose::EULER);

    poseQuat.setRandom();
    poseEuler.setRandom();

    testNotEqual(poseQuat, poseEuler);

    poseEuler = poseQuat;

    testEqual(poseQuat, poseEuler);
}

TEST(RigidBodyPoseTest, conversionTest)
{
    RigidBodyPose poseQuat(RigidBodyPose::QUAT);
    RigidBodyPose poseEuler(RigidBodyPose::EULER);

    poseQuat.setRandom();
    poseEuler.setRandom();
    poseEuler.setFromEulerAnglesXyz(poseQuat.getEulerAnglesXyz());
    poseEuler.position() = poseQuat.position();

    testEqual(poseQuat, poseEuler);

    poseQuat.setRandom();
    poseEuler.setRandom();
    poseEuler.setFromRotationQuaternion(poseQuat.getRotationQuaternion());
    poseEuler.position() = poseQuat.position();

    testEqual(poseQuat, poseEuler);

    poseQuat.setIdentity();
    poseEuler.setIdentity();

    testEqual(poseQuat, poseEuler);
}


TEST(RigidBodyPoseTest, gravityTest)
{
    RigidBodyPose poseQuat(RigidBodyPose::QUAT);
    RigidBodyPose poseEuler(RigidBodyPose::EULER);

    poseQuat.setRandom();
    poseEuler.setRandom();
    poseQuat = poseEuler;

    auto gQuat6d = poseQuat.computeGravityB6D();
    auto gEuler6d = poseEuler.computeGravityB6D();

    ASSERT_TRUE(gQuat6d.head<3>().isApprox(Eigen::Vector3d::Zero()));
    ASSERT_TRUE(gEuler6d.head<3>().isApprox(Eigen::Vector3d::Zero()));

    // test for orientation 0 0 0
    poseQuat.setIdentity();
    poseEuler.setIdentity();

    auto gQuat = poseQuat.computeGravityB();
    auto gEuler = poseEuler.computeGravityB();

    Eigen::Vector3d gravityW;
    gravityW << 0.0, 0.0, -9.81;

    ASSERT_TRUE(gQuat.isApprox(gravityW));
    ASSERT_TRUE(gEuler.isApprox(gravityW));

    // test for 180° rotations
    for (size_t i = 0; i < 3; i++)
    {
        poseQuat.setIdentity();
        poseEuler.setIdentity();

        // perturb one rotation to 180°
        Eigen::Vector3d orientationXyz(Eigen::Vector3d::Zero());
        orientationXyz(i) = M_PI;

        poseQuat.setFromEulerAnglesXyz(orientationXyz);
        poseEuler.setFromEulerAnglesXyz(orientationXyz);

        // if we perturb in x or y gravity should point upwards
        if (i < 2)
        {
            Eigen::Vector3d gravityUp;
            gravityUp << 0.0, 0.0, 9.81;

            ASSERT_TRUE(poseQuat.computeGravityB().isApprox(gravityUp));
            ASSERT_TRUE(poseEuler.computeGravityB().isApprox(gravityUp));
        }
        else
        {
            // otherwise gravity is normal
            Eigen::Vector3d gravityDown;
            gravityDown << 0.0, 0.0, -9.81;

            ASSERT_TRUE(poseQuat.computeGravityB().isApprox(gravityDown));
            ASSERT_TRUE(poseEuler.computeGravityB().isApprox(gravityDown));
        }
    }

    // test for 30° roll
    poseQuat.setIdentity();
    poseEuler.setIdentity();

    Eigen::Vector3d orientationXyz(Eigen::Vector3d::Zero());
    orientationXyz(0) = 30.0 / 180.0 * M_PI;

    poseQuat.setFromEulerAnglesXyz(orientationXyz);
    poseEuler.setFromEulerAnglesXyz(orientationXyz);

    // x component should be zero
    ASSERT_NEAR(poseQuat.computeGravityB()(0), 0.0, 1e-10);
    ASSERT_NEAR(poseEuler.computeGravityB()(0), 0.0, 1e-10);

    // y component should be negative
    ASSERT_LT(poseQuat.computeGravityB()(1), 0.0);
    ASSERT_LT(poseEuler.computeGravityB()(1), 0.0);

    // z component should be negative
    ASSERT_LT(poseQuat.computeGravityB()(2), 0.0);
    ASSERT_LT(poseEuler.computeGravityB()(2), 0.0);
}

TEST(RigidBodyPoseTest, rotationTest)
{
    RigidBodyPose poseQuat(RigidBodyPose::QUAT);
    RigidBodyPose poseEuler(RigidBodyPose::EULER);

    Eigen::Vector3d testVector;
    testVector.setRandom();

    Eigen::Vector3d rotatedVectorQuat;
    Eigen::Vector3d rotatedVectorEigen;

    Eigen::Vector3d baseVectorQuat = poseQuat.rotateBaseToInertia(testVector);
    Eigen::Vector3d baseVectorEuler = poseEuler.rotateBaseToInertia(testVector);

    Eigen::Vector3d baseVectorQuatRotMatrix = poseQuat.getRotationMatrix().toImplementation() * testVector;
    Eigen::Vector3d baseVectorEulerRotMatrix = poseEuler.getRotationMatrix().toImplementation() * testVector;

    ASSERT_TRUE(baseVectorQuat.isApprox(baseVectorEuler, 1e-10));
    ASSERT_TRUE(baseVectorQuat.isApprox(baseVectorQuatRotMatrix, 1e-10));
    ASSERT_TRUE(baseVectorQuat.isApprox(baseVectorEulerRotMatrix, 1e-10));

    ASSERT_TRUE(poseQuat.rotateInertiaToBase(baseVectorQuat).isApprox(testVector, 1e-10));
    ASSERT_TRUE(poseEuler.rotateInertiaToBase(baseVectorEuler).isApprox(testVector, 1e-10));
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
