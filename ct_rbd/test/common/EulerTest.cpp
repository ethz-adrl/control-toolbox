
/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/rbd/rbd.h>
#include <gtest/gtest.h>

bool quatEqual(const Eigen::Quaterniond q1, const Eigen::Quaterniond q2)
{
    Eigen::Quaterniond q2mirrored;
    q2mirrored.coeffs() = -q2.coeffs();
    return q1.isApprox(q2) || q1.isApprox(q2mirrored); 
}


TEST(EulerAnglesTest, EulerAnglesTest)
{
    using namespace ct::rbd;

    size_t nTests = 100;
    for (size_t i = 0; i < nTests; i++)
    {
        // test construction from Eigen vector
        {
            Eigen::Vector3d v = Eigen::Vector3d::Random();
            EulerAnglesXYZ e(v);
            ASSERT_TRUE(e.isApprox(v));
        }
        // test construction from three scalars
        {
            Eigen::Vector3d v = Eigen::Vector3d::Random();
            EulerAnglesXYZ e(v(0), v(1), v(2));
            ASSERT_TRUE(e.isApprox(v));
        }
        // test construction from rotation matrix
        {
            Eigen::Quaterniond q(Eigen::Vector4d::Random());
            q.normalize();
            auto r = q.toRotationMatrix();
            EulerAnglesXYZ e(r);
            auto e_ref = r.eulerAngles(0, 1, 2);
            ASSERT_TRUE(e_ref.isApprox(e));
        }
        // test construction from quaternion
        {
            Eigen::Quaterniond q(Eigen::Vector4d::Random());
            q.normalize();
            EulerAnglesXYZ e(q);
            auto e_ref = q.toRotationMatrix().eulerAngles(0, 1, 2);
            ASSERT_TRUE(e_ref.isApprox(e));
        }

        // test assignment from Eigen-vector
        {
            Eigen::Vector3d v = Eigen::Vector3d::Random();
            EulerAnglesXYZ e;
            e = v;
            ASSERT_TRUE(e.isApprox(v));
        }
        // test assignment from quaternion
        {
            Eigen::Quaterniond q(Eigen::Vector4d::Random());
            q.normalize();
            EulerAnglesXYZ e;
            e = q;
            auto e_ref = q.toRotationMatrix().eulerAngles(0, 1, 2);
            ASSERT_TRUE(e_ref.isApprox(e));
        }
        // test assignment from rotation matrix
        {
            Eigen::Quaterniond q(Eigen::Vector4d::Random());
            auto r = q.toRotationMatrix();
            EulerAnglesXYZ e;
            e = r;
            auto e_ref = r.eulerAngles(0, 1, 2);
            ASSERT_TRUE(e_ref.isApprox(e));
        }
        // test assignment from other euler angles
        {
            EulerAnglesXYZ e1, e2;
            e1.setRandom();
            e2 = e1;
            ASSERT_TRUE(e1.isApprox(e2));
        }
        // test angles accessor
        {
            EulerAnglesXYZ e1, e2;
            e1.setRandom();
            e2 = e1;
            ASSERT_TRUE(e1.angles().isApprox(e2));
        }
        // test inverse
        {
            EulerAnglesXYZ e1, e2;
            e1.setRandom();
            e2 = -e1;
            ASSERT_TRUE(e1.isApprox(e2.inverse()));
        }

        // test conversion to quaternion
        {
            Eigen::Quaterniond q(Eigen::Vector4d::Random());
            q.normalize();
            EulerAnglesXYZ e;
            e = q;
            auto q2 = e.toQuaternion();
            ASSERT_TRUE(quatEqual(q, q2));
        }
        // test conversion to rotation matrix
        {
            Eigen::Quaterniond q(Eigen::Vector4d::Random());
            q.normalize();
            auto r = q.toRotationMatrix();
            EulerAnglesXYZ e;
            e = r;
            auto r2 = e.toRotationMatrix();
            ASSERT_TRUE(r.isApprox(r2));
        }
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}