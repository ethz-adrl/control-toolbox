/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/core/core.h>

#include <gtest/gtest.h>
#include <type_traits>

using namespace ct::core;

TEST(StateManifoldTest, EuclideanStateDynamicTest)
{
    {
        EuclideanStateXd x;
        ASSERT_EQ(x.rows(), 0);
    }
    {
        EuclideanStateXd x, y;
        x.resize(3);
        ASSERT_EQ(x.rows(), 3);
        x.resize(6);
        ASSERT_EQ(x.rows(), 6);

        y = x;
        ASSERT_EQ(y.rows(), 6);
    }

    // TODO: bigger problem ... what to do with the TangentDim parameter???? :-O
}

TEST(StateManifoldTest, StateManifoldTest)
{
    // test construction of a manifold state
    using ManifoldStateD_t = ManifoldState<manif::SE3, manif::SE3Tangent, double>;
    ManifoldStateD_t se3;
    ASSERT_TRUE(ManifoldStateD_t::TangentDim == 6);
    static_assert(std::is_same<double, ManifoldStateD_t::Scalar>::value, "");

    // test construction of a euclidean state
    using EuclideanStateD_t = EuclideanState<6, double>;
    EuclideanStateD_t R6;
    ASSERT_TRUE(EuclideanStateD_t::TangentDim == 6);
    static_assert(std::is_same<double, EuclideanStateD_t::Scalar>::value, "");

    // check if redefinition of euclidean state as float-type works
    using EuclideanStateF_t = EuclideanStateD_t::RedefineScalar<float>;
    EuclideanStateF_t euclideanFloat;
    static_assert(std::is_same<float, EuclideanStateF_t::Scalar>::value, "");

    // check if redefinition of euclidean state as autodiff-type works
    using EuclideanStateADCG_t = EuclideanStateD_t::RedefineScalar<ADCGScalar>;
    EuclideanStateADCG_t euclideanADCG;
    static_assert(std::is_same<ADCGScalar, EuclideanStateADCG_t::Scalar>::value, "");

    // check if redefinition of manifold as float-type works
    using ManifoldStateF_t = ManifoldStateD_t::RedefineScalar<float>;
    ManifoldStateF_t manifoldFloat;
    static_assert(std::is_same<float, ManifoldStateF_t::Scalar>::value, "");

    // check if redefinition of manifold as autodiff-type works
    using ManifoldStateADCG_t = ManifoldStateD_t::RedefineScalar<ADCGScalar>;
    ManifoldStateADCG_t manifoldADCG;
    static_assert(std::is_same<ADCGScalar, ManifoldStateADCG_t::Scalar>::value, "");
}

TEST(ManifoldTest, ManifoldTraitsTest)
{
    using namespace ct::core;
    using Manifold = ManifoldState<manif::SE3, manif::SE3Tangent, double>;
    const int control_dim = 6;

    static_assert(is_euclidean<Manifold>::value == false, "");
    static_assert(is_euclidean<double>::value == false, "");
    static_assert(is_euclidean<std::string>::value == false, "");

    static_assert(is_euclidean<EuclideanState<2>>::value == true, "");
    static_assert(is_euclidean<EuclideanState<control_dim, double>>::value == true, "");

    static_assert(is_real_euclidean<EuclideanState<2>>::value == true, "");
    static_assert(is_real_euclidean<EuclideanState<control_dim, double>>::value == true, "");

    static_assert(is_real_euclidean<EuclideanState<2, float>>::value, "");
    static_assert(is_real_euclidean<EuclideanState<control_dim, float>>::value == true, "");

    static_assert(is_real_euclidean<EuclideanState<control_dim, int>>::value == false, "");
    static_assert(is_real_euclidean<Manifold>::value == false, "");

    static_assert(is_real_euclidean<EuclideanState<2, ct::core::ADCGScalar>>::value == false, "");

    static_assert(is_euclidean<EuclideanStateSymplectic<1, 1, double>>::value == true, "");
    static_assert(is_euclidean<EuclideanStateSymplectic<1, 1, float>>::value == true, "");
    static_assert(is_real_euclidean<EuclideanStateSymplectic<1, 1, double>>::value == true, "");
    static_assert(is_real_euclidean<EuclideanStateSymplectic<1, 1, float>>::value == true, "");

    static_assert(is_symplectic<Manifold>::value == false, "");
    static_assert(is_symplectic<EuclideanState<2>>::value == false, "");
    static_assert(is_symplectic<EuclideanStateSymplectic<1, 1>>::value == true, "");

    ASSERT_TRUE(true);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
