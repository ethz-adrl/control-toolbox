
#include <ct/core/core.h>
#include <gtest/gtest.h>
#include <manif/manif.h>

using namespace ct::core;

const size_t tangent_dim = 3;
const size_t control_dim = 3;

using ManifoldState_t = ManifoldState<manif::SE2d, manif::SE2Tangentd>;

const StateVector<tangent_dim> x_test = StateVector<tangent_dim>::Random();
const ManifoldState_t m_test = ManifoldState_t::Random();
const ControlVector<control_dim> u_test = ControlVector<control_dim>::Random();

Eigen::Matrix3d A_test, B_test;


class EuclideanLTITestSystem final : public LTISystem<EuclideanState<tangent_dim>, control_dim>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EuclideanLTITestSystem()
    {
        this->A() = A_test;
        this->B() = B_test;
    }
};

class SE2LTITestSystem final : public LTISystem<ManifoldState_t, control_dim>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SE2LTITestSystem()
    {
        this->A() << A_test;
        this->B() << B_test;
    }
};


TEST(LTISystemTest, euclideanTest)
{
    static_assert(EuclideanState<tangent_dim>::TangentDim == 3, "");

    EuclideanLTITestSystem sys;

    ASSERT_TRUE(sys.A().isApprox(A_test));
    ASSERT_TRUE(sys.B().isApprox(B_test));

    ASSERT_TRUE(sys.getDerivativeState(x_test, u_test).isApprox(A_test));
    ASSERT_TRUE(sys.getDerivativeControl(x_test, u_test).isApprox(B_test));

    StateVector<tangent_dim> test_derivative;
    sys.computeDynamics(x_test, 0.0, test_derivative);
    ASSERT_TRUE(test_derivative.isApprox(A_test * x_test));
    sys.computeControlledDynamics(x_test, 0.0, u_test, test_derivative);
    ASSERT_TRUE(test_derivative.isApprox(A_test * x_test + B_test * u_test));
}

TEST(LTISystemTest, manifoldTest)
{
    // check dimensions
    static_assert(ManifoldState_t::TangentDim == 3, "");

    SE2LTITestSystem sys;

    ASSERT_TRUE(sys.A().isApprox(A_test));
    ASSERT_TRUE(sys.B().isApprox(B_test));

    ASSERT_TRUE(sys.getDerivativeState(m_test, u_test).isApprox(A_test));
    ASSERT_TRUE(sys.getDerivativeControl(m_test, u_test).isApprox(B_test));

    ManifoldState_t::Tangent test_derivative;
    sys.computeDynamics(m_test, 0.0, test_derivative);
    ASSERT_TRUE(test_derivative.isApprox((A_test * m_test.log()), 1e-6));
    sys.computeControlledDynamics(m_test, 0.0, u_test, test_derivative);
    ASSERT_TRUE(test_derivative.isApprox((A_test * m_test.log() + B_test * u_test), 1e-6));
}


int main(int argc, char** argv)
{
    A_test << 0, 1, 0, 0, 0, 1, 0, 0, 0;
    B_test << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
