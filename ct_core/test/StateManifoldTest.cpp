/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/core/core.h>

#include <gtest/gtest.h>

using namespace ct::core;

TEST(StateManifoldTest, StateManifoldTest)
{
    using State_t = ManifoldState<manif::SE3<double>, manif::SE3Tangent<double>>;
    State_t se3;
    ASSERT_TRUE(State_t::TangentDim == 6);

    using EuclideanState_t = StateVector<6>;
    EuclideanState_t R6;
    ASSERT_TRUE(EuclideanState_t::TangentDim == 6);
}

TEST(ManifoldTest, ManifoldTraitsTest)
{
    using namespace ct::core;
    using Manifold = ManifoldState<manif::SE3d, manif::SE3Tangentd>;
    const size_t control_dim = 6;

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
