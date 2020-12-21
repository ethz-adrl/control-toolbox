
#include <ct/core/core.h>
#include <gtest/gtest.h>

#include "LinearTestSystems.h"

using namespace ct::core;


TEST(LTISystemTest, euclideanTest)
{
    const EuclideanState<tangent_dim> x_test = StateVector<tangent_dim>::Random();
    const ControlVectord u_test = ControlVectord::Random(control_dim);

    static_assert(EuclideanState<tangent_dim>::TangentDim == 3, "");

    // test without artificial offset - continous time
    // {
    //     ContEuclideanLTITestSystem sys(A_test, B_test, EuclideanState<tangent_dim>::NeutralElement(), ControlVectord::Zero(3));

    //     // check if getters work
    //     ASSERT_TRUE(sys.A().isApprox(A_test));
    //     ASSERT_TRUE(sys.B().isApprox(B_test));
    //     ASSERT_TRUE(sys.getDerivativeState().isApprox(A_test));
    //     ASSERT_TRUE(sys.getDerivativeControl().isApprox(B_test));

    //     //check if dynamics computation behaves as expected
    //     StateVector<tangent_dim> test_derivative;
    //     sys.computeDynamics(x_test, 0.0, test_derivative);
    //     ASSERT_TRUE(test_derivative.isApprox(A_test * x_test));
    //     //ASSERT_TRUE(test_derivative.isApprox(A_test * sys.lift(x_test)));
    //     sys.computeControlledDynamics(x_test, 0.0, u_test, test_derivative);
    //     ASSERT_TRUE(test_derivative.isApprox(A_test * x_test + B_test * u_test));
    //    // ASSERT_TRUE(test_derivative.isApprox(A_test * sys.lift(x_test) + B_test * u_test));
    // }

    // // test with artificial offset during lifting - continous time
    // {
    //     ContEuclideanLTITestSystem sys(A_test, B_test, EuclideanState<tangent_dim>::Random());

    //     // check if getters work
    //     ASSERT_TRUE(sys.A().isApprox(A_test));
    //     ASSERT_TRUE(sys.B().isApprox(B_test));
    //     ASSERT_TRUE(sys.getDerivativeState(x_test, u_test).isApprox(A_test));
    //     ASSERT_TRUE(sys.getDerivativeControl(x_test, u_test).isApprox(B_test));

    //     //check if dynamics computation behaves as expected
    //     StateVector<tangent_dim> test_derivative;
    //     sys.computeDynamics(x_test, 0.0, test_derivative);
    //     ASSERT_FALSE(test_derivative.isApprox(A_test * x_test));
    //     ASSERT_TRUE(test_derivative.isApprox(A_test * sys.lift(x_test)));

    //     sys.computeControlledDynamics(x_test, 0.0, u_test, test_derivative);
    //     ASSERT_FALSE(test_derivative.isApprox(A_test * x_test + B_test * u_test));
    //     ASSERT_TRUE(test_derivative.isApprox(A_test * sys.lift(x_test) + B_test * u_test));
    // }

    // // test without artificial offset during lifting - discrete time
    // {
    //     DiscrEuclideanLTITestSystem sys(A_test, B_test, EuclideanState<tangent_dim>::NeutralElement());

    //     // check if getters work
    //     ASSERT_TRUE(sys.A().isApprox(A_test));
    //     ASSERT_TRUE(sys.B().isApprox(B_test));
    //     ASSERT_TRUE(sys.getDerivativeState(x_test, u_test).isApprox(A_test));
    //     ASSERT_TRUE(sys.getDerivativeControl(x_test, u_test).isApprox(B_test));

    //     //check if dynamics computation behaves as expected
    //     // @WARNING note the increment formulation
    //     // x_{n+1} = x_n + A*x_n + B*u
    //     StateVector<tangent_dim> state_incr;
    //     sys.computeDynamics(x_test, 0, state_incr);
    //     ASSERT_FALSE(state_incr.isApprox(A_test * x_test));
    //     ASSERT_TRUE(state_incr.isApprox((A_test - Eigen::Matrix3d::Identity()) * x_test));
    //     ASSERT_TRUE((state_incr + x_test).isApprox(A_test * x_test));
    //     ASSERT_TRUE(state_incr.isApprox((A_test - Eigen::Matrix3d::Identity()) * sys.lift(x_test)));
    //     sys.computeControlledDynamics(x_test, 0, u_test, state_incr);
    //     ASSERT_FALSE(state_incr.isApprox(A_test * x_test + B_test * u_test));
    //     ASSERT_TRUE(state_incr.isApprox((A_test - Eigen::Matrix3d::Identity()) * x_test + B_test * u_test));
    //     ASSERT_TRUE(state_incr.isApprox((A_test - Eigen::Matrix3d::Identity()) * sys.lift(x_test) + B_test * u_test));
    //     ASSERT_TRUE((state_incr + x_test).isApprox(A_test * x_test + B_test * u_test));
    // }

    // // test with artificial random offset during lifting - discrete time
    // {
    //     DiscrEuclideanLTITestSystem sys(A_test, B_test, EuclideanState<tangent_dim>::Random());

    //     // check if getters work
    //     ASSERT_TRUE(sys.A().isApprox(A_test));
    //     ASSERT_TRUE(sys.B().isApprox(B_test));
    //     ASSERT_TRUE(sys.getDerivativeState(x_test, u_test).isApprox(A_test));
    //     ASSERT_TRUE(sys.getDerivativeControl(x_test, u_test).isApprox(B_test));

    //     //check if dynamics computation behaves as expected
    //     StateVector<tangent_dim> state_incr;
    //     sys.computeDynamics(x_test, 0.0, state_incr);
    //     ASSERT_FALSE(state_incr.isApprox(A_test * x_test));
    //     ASSERT_FALSE(state_incr.isApprox((A_test - Eigen::Matrix3d::Identity()) * x_test));  // false, need to lift
    //     ASSERT_TRUE(state_incr.isApprox((A_test - Eigen::Matrix3d::Identity()) * sys.lift(x_test)));  // correct
    //     ASSERT_TRUE((state_incr + sys.lift(x_test)).isApprox(A_test * sys.lift(x_test)));

    //     sys.computeControlledDynamics(x_test, 0.0, u_test, state_incr);
    //     ASSERT_FALSE(state_incr.isApprox(A_test * x_test + B_test * u_test));
    //     ASSERT_FALSE(state_incr.isApprox((A_test - Eigen::Matrix3d::Identity()) * x_test + B_test * u_test));
    //     ASSERT_TRUE(state_incr.isApprox((A_test - Eigen::Matrix3d::Identity()) * sys.lift(x_test) + B_test * u_test));
    //     ASSERT_TRUE((state_incr + sys.lift(x_test)).isApprox(A_test * sys.lift(x_test) + B_test * u_test));
    // }
}


#ifdef CT_USE_MANIF

TEST(ContSE2LTISystemTest, TrivialLinearizationPointWorks)
{
    // stable linear dynamics, amplification 1 ensures we can reach destinations in one integration step. Control input matrix is a direct pass-through.
    Eigen::Matrix3d A_test, B_test;
    A_test = -Eigen::Matrix3d::Identity();
    B_test = Eigen::Matrix3d::Identity();

    const ManifoldState_t m0 = ManifoldState_t::Identity();
    const ControlVectord u0 = ControlVectord::Zero(control_dim);

    // check dimensions
    static_assert(EuclideanState<tangent_dim>::TangentDim == 3, "");  // TODO: why does the dynamic assert fail to link?
    //ASSERT_EQ(ManifoldState_t::TangentDim, 3);
    ASSERT_EQ(control_dim, 3);

    ContSE2LTITestSystem lti(A_test, B_test, m0, u0);

    // Getters work
    EXPECT_TRUE(lti.A().isApprox(A_test));
    EXPECT_TRUE(lti.B().isApprox(B_test));
    EXPECT_TRUE(lti.getDerivativeState().isApprox(A_test));
    EXPECT_TRUE(lti.getDerivativeControl().isApprox(B_test));

    // Dynamics computation results in zero at linearization point
    ManifoldState_t::Tangent dxdt;
    lti.computeDynamics(m0, 0.0, dxdt);
    EXPECT_TRUE(dxdt.isApprox(ControlVectord::Zero(control_dim)));
    lti.computeControlledDynamics(m0, 0.0, u0, dxdt);
    EXPECT_TRUE(dxdt.isApprox(ControlVectord::Zero(control_dim)));
}

TEST(ContSE2LTISystemTest, DerivativesWorkInNontrivialLinearizationPoint)
{
    // stable linear dynamics, amplification 1 ensures we can reach destinations in one integration step. Control input matrix is a direct pass-through.
    Eigen::Matrix3d A_test, B_test;
    A_test = -Eigen::Matrix3d::Identity();
    B_test = Eigen::Matrix3d::Identity();

    // Select new linearization point, offset from zero and non-trivial.
    ManifoldState_t m_lin = ManifoldState_t::Identity();
    m_lin.coeffs() << 1.0, 1.0, sqrt(2) / 2, sqrt(2) / 2;
    const ControlVectord u_lin = ControlVectord::Zero(control_dim);

    ContSE2LTITestSystem lti(A_test, B_test, m_lin, u_lin);

    // Getters work
    EXPECT_TRUE(lti.A().isApprox(A_test));
    EXPECT_TRUE(lti.B().isApprox(B_test));
    EXPECT_TRUE(lti.getDerivativeState().isApprox(A_test));
    EXPECT_TRUE(lti.getDerivativeControl().isApprox(B_test));

    // Test a case where the derivative points directly to the reference point.
    auto m_other = ManifoldState_t::Random();
    ManifoldState_t::Tangent dxdt;
    lti.computeControlledDynamics(m_other, 0.0, u_lin, dxdt);
    EXPECT_TRUE(m_lin.isApprox(m_other + dxdt));


    // Construct a special case constructed here, in which A() is set to zero and the entire dynamics governed by the control input.
    // Choose the control vector u that drives the system to m_lin - it is equal to test_derivative from above.
    lti.A().setZero();
    ControlVectord u_opt = dxdt.toImplementation();
    ManifoldState_t::Tangent dxdt_control_only;
    lti.computeControlledDynamics(m_other, 0.0, u_opt, dxdt_control_only);
    // the derivative should drive us straight to the target state.
    EXPECT_TRUE(m_lin.isApprox(m_other + dxdt_control_only));
}

TEST(ContSE2LTISystemTest, WorksWithNonZeroControlSetpoint)
{
    // empty dynamics. Control input matrix is a direct pass-through.
    Eigen::Matrix3d A_test, B_test;
    A_test = Eigen::Matrix3d::Zero();
    B_test = Eigen::Matrix3d::Identity();

    // Select new reference manifold point, offset from zero and non-trivial.
    ManifoldState_t m_lin = ManifoldState_t::Identity();
    m_lin.coeffs() << 1.0, 1.0, sqrt(2) / 2, sqrt(2) / 2;
    // And a second manifold point, which is random.
    ManifoldState_t m_other = ManifoldState_t::Random();

    // design a reference control centered in m_lin, which drives the system back from m_other in one step.
    const ControlVectord u_lin = m_other.rminus(m_lin).toImplementation();

    ContSE2LTITestSystem lti(A_test, B_test, m_lin, u_lin);

    // Getters work
    EXPECT_TRUE(lti.A().isApprox(A_test));
    EXPECT_TRUE(lti.B().isApprox(B_test));
    EXPECT_TRUE(lti.getDerivativeState().isApprox(A_test));
    EXPECT_TRUE(lti.getDerivativeControl().isApprox(B_test));

    // the derivative should drive us straight to the target state from m_other with A=0 and u=0, all due to feed-forward.
    ManifoldState_t::Tangent dxdt;
    lti.computeControlledDynamics(m_other, 0.0, ControlVectord::Zero(3), dxdt);
    EXPECT_TRUE(m_lin.isApprox(m_other + dxdt));
}

// test with reference being at a random point
// {
//     ContSE2LTITestSystem sys(A_test, B_test, ManifoldState_t::Random(), ControlVectord::Random(control_dim));

//     // check if getters work
//     ASSERT_TRUE(sys.A().isApprox(A_test));
//     ASSERT_TRUE(sys.B().isApprox(B_test));
//     ASSERT_TRUE(sys.getDerivativeState().isApprox(A_test));
//     ASSERT_TRUE(sys.getDerivativeControl().isApprox(B_test));

//     //check if dynamics computation behaves as expected
//     ManifoldState_t::Tangent test_derivative;
//     sys.computeDynamics(m_test, 0.0, test_derivative);
//     ASSERT_FALSE(test_derivative.isApprox((A_test * m_test.log()), 1e-6));
//     // ASSERT_TRUE(test_derivative.isApprox((A_test * sys.lift(m_test)), 1e-6));

//     sys.computeControlledDynamics(m_test, 0.0, u_test, test_derivative);
//     ASSERT_FALSE(test_derivative.isApprox((A_test * m_test.log() + B_test * u_test), 1e-6));
//     //ASSERT_TRUE(test_derivative.isApprox((A_test * sys.lift(m_test) + B_test * u_test), 1e-6));
// }

#endif  // CT_USE_MANIF


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
