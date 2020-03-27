
#include <ct/core/core.h>

//#include <Eigen/Dense>
//#include <ct/core/systems/continuous_time/System.h>
//#include <ct/core/systems/continuous_time/System-impl.h>
//#include <ct/core/systems/continuous_time/ControlledSystem.h>
//#include <ct/core/systems/continuous_time/ControlledSystem-impl.h>
//#include <ct/core/types/EuclideanState.h>

#include <gtest/gtest.h>
#include "TestNonlinearSystem.h"

using namespace ct::core;

TEST(ControlledSystemTest, instantiationTests)
{
    const size_t tangent_dim = 2;
    const size_t control_dim = 1;
    const EuclideanState<tangent_dim> x_test = EuclideanState<tangent_dim>::Random();
    const ControlVector<control_dim> u_test = ControlVector<control_dim>::Random();

    static_assert(EuclideanState<tangent_dim>::TangentDim == 2, "");

    // test continous time instantiation using double
    {
        const bool cont_time = true;
        TestNonlinearSystem<cont_time> sys(1.0);

        // make sure the imprinted time type is correct
        static_assert(
            std::is_same<TestNonlinearSystem<cont_time>::Time_t, double>::value, "Check for correct time type");

        //check if dynamics computation behaves as expected
        EuclideanState<tangent_dim>::Tangent test_derivative;
        sys.computeDynamics(x_test, 0.0, test_derivative);
        ASSERT_FALSE(eigen_is_nan(test_derivative));

        sys.computeControlledDynamics(x_test, 0.0, u_test, test_derivative);
        ASSERT_FALSE(eigen_is_nan(test_derivative));
    }

    // test continous time instantiation using float
    {
        const bool cont_time = true;
        tpl::TestNonlinearSystem<float, cont_time> sys(1.0);

        // make sure the imprinted time type is correct
        static_assert(std::is_same<tpl::TestNonlinearSystem<float, cont_time>::Time_t, float>::value,
            "Check for correct time type");
    }

    // test discrete time instantiation using int
    {
        const bool cont_time = false;
        TestNonlinearSystem<cont_time> sys(1.0);

        // make sure the imprinted time type is correct
        static_assert(std::is_same<TestNonlinearSystem<cont_time>::Time_t, int>::value, "Check for correct time type");

        //check if dynamics computation behaves as expected
        EuclideanState<tangent_dim>::Tangent dx;
        sys.computeDynamics(x_test, 0, dx);
        ASSERT_FALSE(eigen_is_nan(dx));

        sys.computeControlledDynamics(x_test, 0, u_test, dx);
        ASSERT_FALSE(eigen_is_nan(dx));
    }
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
