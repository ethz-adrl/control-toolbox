
#include <ct/core/core.h>
#include "TestNonlinearSystem.h"

// Bring in gtest
#include <gtest/gtest.h>

using namespace ct::core;

const size_t tangent_dim = 3;
const size_t control_dim = 3;

using ManifoldState_t = ManifoldState<manif::SE2d, manif::SE2Tangentd>;
using EuclideanState_t = EuclideanState<tangent_dim>;

const StateVector<tangent_dim> x_test = StateVector<tangent_dim>::Random();
const ManifoldState_t m_test = ManifoldState_t::Random();
const ControlVector<control_dim> u_test = ControlVector<control_dim>::Random();

Eigen::Matrix3d A_test, B_test;


class EuclideanLTITestSystem final : public LTISystem<EuclideanState_t, control_dim>
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


TEST(SystemLinearizerTest, EuclideanTest)
{
    std::shared_ptr<ControlledSystem<EuclideanState_t, control_dim>> testSystem(new EuclideanLTITestSystem());

    for (bool doubleSided : {false, true})
    {
        SystemLinearizer<EuclideanState_t, control_dim> linearizer(testSystem, doubleSided);

        StateMatrix<tangent_dim> A;
        StateControlMatrix<tangent_dim, control_dim> B;
        linearizer.getDerivatives(A, B, x_test, u_test);

        ASSERT_TRUE(A.isApprox(A_test, 1e-6));
        ASSERT_TRUE(B.isApprox(B_test, 1e-6));
    }
}


TEST(SystemLinearizerTest, Se2Test)
{
    std::shared_ptr<ControlledSystem<ManifoldState_t, control_dim>> testSystem(new SE2LTITestSystem());

    for (bool doubleSided : {false, true})
    {
        SystemLinearizer<ManifoldState_t, control_dim> linearizer(testSystem, doubleSided);

        StateMatrix<tangent_dim> A = linearizer.getDerivativeState(m_test, u_test);
        StateControlMatrix<tangent_dim, control_dim> B = linearizer.getDerivativeControl(m_test, u_test);

        //std::cout << A << std::endl;
        //std::cout << B << std::endl;

        ASSERT_TRUE(A.isApprox(A_test, 1e-7));
        ASSERT_TRUE(B.isApprox(B_test, 1e-7));
    }
}


int main(int argc, char** argv)
{
    A_test << 0, 1, 0, 0, 0, 1, 0, 0, 0;
    B_test << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
