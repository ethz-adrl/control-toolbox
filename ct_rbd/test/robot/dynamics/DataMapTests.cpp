/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <memory>
#include <array>

#include <gtest/gtest.h>

#include <ct/rbd/rbd.h>

#include <ct/rbd/robot/kinematics/RBDDataMap.h>
#include "../../models/testhyq/RobCoGenTestHyQ.h"

using namespace ct::rbd;

TEST(DataMapTestsHyQ, datamap_test_bool)
{
    RBDDataMap<bool, TestHyQ::RobCoGenContainer::NLINKS> ldm_a;
    RBDDataMap<bool, TestHyQ::RobCoGenContainer::NLINKS> ldm_b;

    for (size_t id = 0; id < TestHyQ::RobCoGenContainer::NLINKS; ++id)
    {
        ldm_a[id] = true;
        ldm_b[id] = false;
    }

    // check all values
    for (size_t id = 0; id < TestHyQ::RobCoGenContainer::NLINKS; ++id)
    {
        ASSERT_TRUE(ldm_a[id] == true);
        ASSERT_TRUE(ldm_b[id] == false);
        ldm_a[id] = ldm_b[id];
        ASSERT_FALSE(ldm_a[id] == true);
    }

    // constant assignment
    ldm_b = true;
    ASSERT_TRUE(ldm_b[0] == true);
    ASSERT_TRUE(ldm_b[TestHyQ::RobCoGenContainer::NLINKS - 1] == true);

    // copy from other
    ldm_b = ldm_a;
    ASSERT_TRUE(ldm_b[0] == false);
    ASSERT_TRUE(ldm_b[TestHyQ::RobCoGenContainer::NLINKS - 1] == false);

    // << operator
    std::cout << ldm_b << std::endl;
}

TEST(DataMapTestsHyQ, datamap_test_eigen)
{
    const size_t numEE = TestHyQ::Kinematics::NUM_EE;

    RBDDataMap<Eigen::Vector3d, numEE> ldm_a;
    RBDDataMap<Eigen::Vector3d, numEE> ldm_b;

    for (size_t id = 0; id < numEE; ++id)
    {
        ldm_a[id] = Eigen::Vector3d::Constant(-888.8);
        ldm_b[id] = Eigen::Vector3d::Constant(222.2);
    }

    // check all values
    for (size_t id = 0; id < numEE; ++id)
    {
        ASSERT_TRUE(ldm_a[id](1) == -888.8);
        ASSERT_TRUE(ldm_b[id](1) == 222.2);
        ldm_a[id] = ldm_b[id];
        ASSERT_FALSE(ldm_a[id](1) == -888.8);
    }

    // constant assignment
    ldm_b = Eigen::Vector3d::Constant(-888.8);
    ASSERT_TRUE(ldm_b[0](1) == -888.8);
    ASSERT_TRUE(ldm_b[numEE - 1](1) == -888.8);

    // copy from other
    ldm_b = ldm_a;
    ASSERT_TRUE(ldm_b[0](1) == 222.2);
    ASSERT_TRUE(ldm_b[numEE - 1](1) == 222.2);

    // << operator
    std::cout << ldm_b << std::endl;
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
