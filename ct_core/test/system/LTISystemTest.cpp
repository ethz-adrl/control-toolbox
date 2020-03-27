
#include <ct/core/core.h>
#include <gtest/gtest.h>

#include "LinearTestSystems.h"

using namespace ct::core;

Eigen::Matrix3d A_test, B_test;


TEST(LTISystemTest, euclideanTest)
{
    const EuclideanState<tangent_dim> x_test = StateVector<tangent_dim>::Random();
    const ControlVector<control_dim> u_test = ControlVector<control_dim>::Random();

    static_assert(EuclideanState<tangent_dim>::TangentDim == 3, "");

    // test without artificial offset during lifting - continous time
    {
        ContEuclideanLTITestSystem sys(A_test, B_test, EuclideanState<tangent_dim>::NeutralElement());

        // check if getters work
        ASSERT_TRUE(sys.A().isApprox(A_test));
        ASSERT_TRUE(sys.B().isApprox(B_test));
        ASSERT_TRUE(sys.getDerivativeState(x_test, u_test).isApprox(A_test));
        ASSERT_TRUE(sys.getDerivativeControl(x_test, u_test).isApprox(B_test));

        //check if dynamics computation behaves as expected
        StateVector<tangent_dim> test_derivative;
        sys.computeDynamics(x_test, 0.0, test_derivative);
        ASSERT_TRUE(test_derivative.isApprox(A_test * x_test));
        ASSERT_TRUE(test_derivative.isApprox(A_test * sys.lift(x_test)));
        sys.computeControlledDynamics(x_test, 0.0, u_test, test_derivative);
        ASSERT_TRUE(test_derivative.isApprox(A_test * x_test + B_test * u_test));
        ASSERT_TRUE(test_derivative.isApprox(A_test * sys.lift(x_test) + B_test * u_test));
    }

    // test with artificial offset during lifting - continous time
    {
        ContEuclideanLTITestSystem sys(A_test, B_test, EuclideanState<tangent_dim>::Random());

        // check if getters work
        ASSERT_TRUE(sys.A().isApprox(A_test));
        ASSERT_TRUE(sys.B().isApprox(B_test));
        ASSERT_TRUE(sys.getDerivativeState(x_test, u_test).isApprox(A_test));
        ASSERT_TRUE(sys.getDerivativeControl(x_test, u_test).isApprox(B_test));

        //check if dynamics computation behaves as expected
        StateVector<tangent_dim> test_derivative;
        sys.computeDynamics(x_test, 0.0, test_derivative);
        ASSERT_FALSE(test_derivative.isApprox(A_test * x_test));
        ASSERT_TRUE(test_derivative.isApprox(A_test * sys.lift(x_test)));

        sys.computeControlledDynamics(x_test, 0.0, u_test, test_derivative);
        ASSERT_FALSE(test_derivative.isApprox(A_test * x_test + B_test * u_test));
        ASSERT_TRUE(test_derivative.isApprox(A_test * sys.lift(x_test) + B_test * u_test));
    }

    // test without artificial offset during lifting - discrete time
    {
        DiscrEuclideanLTITestSystem sys(A_test, B_test, EuclideanState<tangent_dim>::NeutralElement());

        // check if getters work
        ASSERT_TRUE(sys.A().isApprox(A_test));
        ASSERT_TRUE(sys.B().isApprox(B_test));
        ASSERT_TRUE(sys.getDerivativeState(x_test, u_test).isApprox(A_test));
        ASSERT_TRUE(sys.getDerivativeControl(x_test, u_test).isApprox(B_test));

        //check if dynamics computation behaves as expected
        // @WARNING note the increment formulation
        // x_{n+1} = x_n + A*x_n + B*u
        StateVector<tangent_dim> state_incr;
        sys.computeDynamics(x_test, 0, state_incr);
        ASSERT_FALSE(state_incr.isApprox(A_test * x_test));
        ASSERT_TRUE(state_incr.isApprox((A_test - Eigen::Matrix3d::Identity()) * x_test));
        ASSERT_TRUE((state_incr + x_test).isApprox(A_test * x_test));
        ASSERT_TRUE(state_incr.isApprox((A_test - Eigen::Matrix3d::Identity()) * sys.lift(x_test)));
        sys.computeControlledDynamics(x_test, 0, u_test, state_incr);
        ASSERT_FALSE(state_incr.isApprox(A_test * x_test + B_test * u_test));
        ASSERT_TRUE(state_incr.isApprox((A_test - Eigen::Matrix3d::Identity()) * x_test + B_test * u_test));
        ASSERT_TRUE(state_incr.isApprox((A_test - Eigen::Matrix3d::Identity()) * sys.lift(x_test) + B_test * u_test));
        ASSERT_TRUE((state_incr + x_test).isApprox(A_test * x_test + B_test * u_test));
    }

    // test with artificial random offset during lifting - discrete time
    {
        DiscrEuclideanLTITestSystem sys(A_test, B_test, EuclideanState<tangent_dim>::Random());

        // check if getters work
        ASSERT_TRUE(sys.A().isApprox(A_test));
        ASSERT_TRUE(sys.B().isApprox(B_test));
        ASSERT_TRUE(sys.getDerivativeState(x_test, u_test).isApprox(A_test));
        ASSERT_TRUE(sys.getDerivativeControl(x_test, u_test).isApprox(B_test));

        //check if dynamics computation behaves as expected
        StateVector<tangent_dim> state_incr;
        sys.computeDynamics(x_test, 0.0, state_incr);
        ASSERT_FALSE(state_incr.isApprox(A_test * x_test));
        ASSERT_FALSE(state_incr.isApprox((A_test - Eigen::Matrix3d::Identity()) * x_test));  // false, need to lift
        ASSERT_TRUE(state_incr.isApprox((A_test - Eigen::Matrix3d::Identity()) * sys.lift(x_test)));  // correct
        ASSERT_TRUE((state_incr + sys.lift(x_test)).isApprox(A_test * sys.lift(x_test))); 

        sys.computeControlledDynamics(x_test, 0.0, u_test, state_incr);
        ASSERT_FALSE(state_incr.isApprox(A_test * x_test + B_test * u_test));
        ASSERT_FALSE(state_incr.isApprox((A_test - Eigen::Matrix3d::Identity()) * x_test + B_test * u_test));
        ASSERT_TRUE(state_incr.isApprox((A_test - Eigen::Matrix3d::Identity()) * sys.lift(x_test) + B_test * u_test));
        ASSERT_TRUE((state_incr + sys.lift(x_test)).isApprox(A_test * sys.lift(x_test) + B_test * u_test));
    }
}


#ifdef CT_USE_MANIF

TEST(LTISystemTest, manifoldTest)
{
    const ManifoldState_t m_test = ManifoldState_t::Random();
    const ControlVector<control_dim> u_test = ControlVector<control_dim>::Random();

    // check dimensions
    static_assert(ManifoldState_t::TangentDim == 3, "");

    // test with reference being at the neutral element epsilon (lift = log())
    {
        ContSE2LTITestSystem sys(A_test, B_test, ManifoldState_t::Identity());

        // check if getters work
        ASSERT_TRUE(sys.A().isApprox(A_test));
        ASSERT_TRUE(sys.B().isApprox(B_test));
        ASSERT_TRUE(sys.getDerivativeState(m_test, u_test).isApprox(A_test));
        ASSERT_TRUE(sys.getDerivativeControl(m_test, u_test).isApprox(B_test));

        //check if dynamics computation behaves as expected
        ManifoldState_t::Tangent test_derivative;
        sys.computeDynamics(m_test, 0.0, test_derivative);
        ASSERT_TRUE(test_derivative.isApprox((A_test * m_test.log()), 1e-6));
        ASSERT_TRUE(test_derivative.isApprox((A_test * sys.lift(m_test)), 1e-6));

        sys.computeControlledDynamics(m_test, 0.0, u_test, test_derivative);
        ASSERT_TRUE(test_derivative.isApprox((A_test * m_test.log() + B_test * u_test), 1e-6));
        ASSERT_TRUE(test_derivative.isApprox((A_test * sys.lift(m_test) + B_test * u_test), 1e-6));
    }

    // test with reference being at a random point
    {
        ContSE2LTITestSystem sys(A_test, B_test, ManifoldState_t::Random());

        // check if getters work
        ASSERT_TRUE(sys.A().isApprox(A_test));
        ASSERT_TRUE(sys.B().isApprox(B_test));
        ASSERT_TRUE(sys.getDerivativeState(m_test, u_test).isApprox(A_test));
        ASSERT_TRUE(sys.getDerivativeControl(m_test, u_test).isApprox(B_test));

        //check if dynamics computation behaves as expected
        ManifoldState_t::Tangent test_derivative;
        sys.computeDynamics(m_test, 0.0, test_derivative);
        ASSERT_FALSE(test_derivative.isApprox((A_test * m_test.log()), 1e-6));
        ASSERT_TRUE(test_derivative.isApprox((A_test * sys.lift(m_test)), 1e-6));

        sys.computeControlledDynamics(m_test, 0.0, u_test, test_derivative);
        ASSERT_FALSE(test_derivative.isApprox((A_test * m_test.log() + B_test * u_test), 1e-6));
        ASSERT_TRUE(test_derivative.isApprox((A_test * sys.lift(m_test) + B_test * u_test), 1e-6));
    }
}
#endif  // CT_USE_MANIF


int main(int argc, char** argv)
{
    A_test << 0, 1, 0, 0, 0, 1, 0, 0, 0;
    B_test << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
