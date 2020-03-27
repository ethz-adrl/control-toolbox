
#include <ct/core/core.h>
#include "LinearTestSystems.h"
#include <gtest/gtest.h>

using namespace ct::core;

Eigen::Matrix3d A_test, B_test;

/**
 * @brief Test if the system linearizer reproduces the correct A and B matrices in the Euclidean case,
 *  where the reference manifold (origin) varies
 */
TEST(SystemLinearizerTest, Euclidean_continuous_time)
{
    const StateVector<tangent_dim> x_test = StateVector<tangent_dim>::Random();
    const ControlVector<control_dim> u_test = ControlVector<control_dim>::Random();

    for (EuclideanState_t ref_state : {EuclideanState_t::Zero(), EuclideanState_t::Ones()})
    {
        std::shared_ptr<ControlledSystem<EuclideanState_t, control_dim, CONTINUOUS_TIME>> testSystem(
            new ContEuclideanLTITestSystem(A_test, B_test, ref_state));

        for (bool doubleSided : {false, true})
        {
            SystemLinearizer<EuclideanState_t, control_dim, CONTINUOUS_TIME> linearizer(testSystem, doubleSided);

            StateMatrix<tangent_dim> A;
            StateControlMatrix<tangent_dim, control_dim> B;
            linearizer.getDerivatives(A, B, x_test, u_test);

            ASSERT_TRUE(A.isApprox(A_test, 1e-6));
            ASSERT_TRUE(B.isApprox(B_test, 1e-6));
        }
    }
}

TEST(SystemLinearizerTest, Euclidean_discrete_time)
{
    const StateVector<tangent_dim> x_test = StateVector<tangent_dim>::Random();
    const ControlVector<control_dim> u_test = ControlVector<control_dim>::Random();

    for (EuclideanState_t ref_state : {EuclideanState_t::Zero(), EuclideanState_t::Ones()})
    {
        std::shared_ptr<ControlledSystem<EuclideanState_t, control_dim, DISCRETE_TIME>> testSystem(
            new DiscrEuclideanLTITestSystem(A_test, B_test, ref_state));

        for (bool doubleSided : {false, true})
        {
            SystemLinearizer<EuclideanState_t, control_dim, DISCRETE_TIME> linearizer(testSystem, doubleSided);

            StateMatrix<tangent_dim> A;
            StateControlMatrix<tangent_dim, control_dim> B;
            linearizer.getDerivatives(A, B, x_test, u_test);

            ASSERT_TRUE(A.isApprox(A_test, 1e-6));
            ASSERT_TRUE(B.isApprox(B_test, 1e-6));
        }
    }
}


#ifdef CT_USE_MANIF
/**
 * @brief Test if the system linearizer reproduces the correct A and B matrices in the Manifold case,
 * where the reference manifold (origin) varies between epsilon and random
 */
TEST(SystemLinearizerTest, Se2_continuous_time)
{
    const ManifoldState_t m_test = ManifoldState_t::Random();
    const ControlVector<control_dim> u_test = ControlVector<control_dim>::Random();

    for (ManifoldState_t ref_manifold : {ManifoldState_t::Identity(), ManifoldState_t::Random()})
    {
        std::shared_ptr<ControlledSystem<ManifoldState_t, control_dim, CONTINUOUS_TIME>> testSystem(
            new ContSE2LTITestSystem(A_test, B_test, ref_manifold));

        for (bool doubleSided : {false, true})
        {
            SystemLinearizer<ManifoldState_t, control_dim, CONTINUOUS_TIME> linearizer(testSystem, doubleSided);

            StateMatrix<tangent_dim> A = linearizer.getDerivativeState(m_test, u_test);
            StateControlMatrix<tangent_dim, control_dim> B = linearizer.getDerivativeControl(m_test, u_test);

            ASSERT_TRUE(A.isApprox(A_test, 1e-7));
            ASSERT_TRUE(B.isApprox(B_test, 1e-7));
        }
    }
}

TEST(SystemLinearizerTest, Se2_discrete_time)
{
    const ManifoldState_t m_test = ManifoldState_t::Random();
    const ControlVector<control_dim> u_test = ControlVector<control_dim>::Random();

    for (ManifoldState_t ref_manifold : {ManifoldState_t::Identity(), ManifoldState_t::Random()})
    {
        std::shared_ptr<ControlledSystem<ManifoldState_t, control_dim, DISCRETE_TIME>> testSystem(
            new DiscrSE2LTITestSystem(A_test, B_test, ref_manifold));

        for (bool doubleSided : {false, true})
        {
            SystemLinearizer<ManifoldState_t, control_dim, DISCRETE_TIME> linearizer(testSystem, doubleSided);

            StateMatrix<tangent_dim> A = linearizer.getDerivativeState(m_test, u_test);
            StateControlMatrix<tangent_dim, control_dim> B = linearizer.getDerivativeControl(m_test, u_test);

            ASSERT_TRUE(A.isApprox(A_test, 1e-7));
            ASSERT_TRUE(B.isApprox(B_test, 1e-7));
        }
    }
}
#endif  // CT_USE_MANIF


int main(int argc, char** argv)
{
    A_test << 0, 1, 0, 0, 0, 1, 1, 0, 0;
    B_test << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
