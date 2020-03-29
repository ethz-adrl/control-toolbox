/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/
#include <iostream>
#include <cstdlib>

#include <ct/core/core.h>
#include <ct/core/types/arrays/ManifArrays.h>

#include <gtest/gtest.h>

using namespace ct::core;

TEST(DiscreteArrayTest, ManifTypeTest)
{
    const size_t nEl = 10;

    // test constructors
    {
        SE3Array arr(nEl);
    }
    {
        SE3Array arr(nEl, manif::SE3d::Identity());
    }
    {
        SE3Array arr(nEl, manif::SE3d::Random());
        SE3Array other(arr);
        ASSERT_TRUE(arr == other);
    }

    // test operators
    {
        SE3Array arr(nEl, manif::SE3d::Random());
        SE3Array other = arr;
        ASSERT_TRUE(arr == other);
        ASSERT_FALSE(arr != other);
    }
    {
        SE3Array arr1(nEl, manif::SE3d::Random());
        SE3Array arr2(nEl, manif::SE3d::Random());
        ASSERT_FALSE(arr1 == arr2);
        ASSERT_TRUE(arr1 != arr2);
    }
}


TEST(DiscreteArrayTest, UnaryPlusMinusTest)
{
    const size_t nEl = 10;
    const size_t state_dim = 2;

    //! create two state vector array and fill them with random elements
    StateVectorArray<state_dim> array1(nEl);
    StateVectorArray<state_dim> array2(nEl);

    for (size_t i = 0; i < nEl; i++)
    {
        array1[i].setRandom();
        array2[i].setRandom();
    }

    ASSERT_FALSE(array1 == array2);

    //! create backups for for later comparison
    StateVectorArray<state_dim> array1_backup = array1;
    StateVectorArray<state_dim> array2_backup = array2;


    //! test the overloaded operators
    StateVectorArray<state_dim> array_sum = array1 + array2;
    StateVectorArray<state_dim> array_diff = array1 - array2;

    for (size_t i = 0; i < nEl; i++)
    {
        //! check that the summation is correct
        ASSERT_EQ(array_sum[i], (array1[i] + array2[i]));

        //! check that the difference is correct
        ASSERT_EQ(array_diff[i], (array1[i] - array2[i]));

        //! check that original elements were not altered
        ASSERT_EQ(array1[i], array1_backup[i]);
        ASSERT_EQ(array2[i], array2_backup[i]);
    }
}

template <typename T>
void runAssignmentTest()
{
    const size_t nEl = 10;
    const size_t state_dim = 2;

    using StateVector_t = StateVectorArray<state_dim, T>;
    //! create state vector array and fill with random elements
    StateVector_t array1(nEl);

    for (size_t i = 0; i < nEl; i++)
        array1[i].setRandom();

    //! test the overloaded operator in two ways
    StateVector_t array2;
    array2 = array1;

    StateVector_t array3 = array1;

    for (size_t i = 0; i < nEl; i++)
    {
        //! check that the elements are equal
        ASSERT_EQ(array1[i], array2[i]);
        ASSERT_EQ(array1[i], array3[i]);
    }

    ASSERT_TRUE(array1 == array2);
    ASSERT_TRUE(array1 == array3);
}

TEST(DiscreteArrayTest, AssignmentTest)
{
    runAssignmentTest<double>();
    runAssignmentTest<float>();
}

TEST(DiscreteArrayTest, AddAssignTest)
{
    const size_t nEl = 10;
    const size_t state_dim = 2;

    //! create two state vector array and fill them with random elements
    StateVectorArray<state_dim> array1(nEl);
    StateVectorArray<state_dim> array2(nEl);

    for (size_t i = 0; i < nEl; i++)
    {
        array1[i].setRandom();
        array2[i].setRandom();
    }

    //! create backups for for later comparison
    StateVectorArray<state_dim> array1_backup = array1;
    StateVectorArray<state_dim> array2_backup = array2;


    //! test the overloaded operator
    array1 += array2;

    for (size_t i = 0; i < nEl; i++)
    {
        //! check that the summation is correct
        ASSERT_EQ(array1[i], (array1_backup[i] + array2_backup[i]));

        //! check that original rhs elements were not altered
        ASSERT_EQ(array2[i], array2_backup[i]);
    }
}


TEST(DiscreteArrayTest, SubstractAssignTest)
{
    const size_t nEl = 10;
    const size_t state_dim = 2;

    //! create two state vector array and fill them with random elements
    StateVectorArray<state_dim> array1(nEl);
    StateVectorArray<state_dim> array2(nEl);

    for (size_t i = 0; i < nEl; i++)
    {
        array1[i].setRandom();
        array2[i].setRandom();
    }

    //! create backups for for later comparison
    StateVectorArray<state_dim> array1_backup = array1;
    StateVectorArray<state_dim> array2_backup = array2;


    //! test the overloaded operator
    array1 -= array2;

    for (size_t i = 0; i < nEl; i++)
    {
        //! check that the summation is correct
        ASSERT_EQ(array1[i], (array1_backup[i] - array2_backup[i]));

        //! check that original rhs elements were not altered
        ASSERT_EQ(array2[i], array2_backup[i]);
    }
}


TEST(DiscreteArrayTest, MultiplicationTest)
{
    const size_t nEl = 10;
    const size_t state_dim = 2;

    StateVectorArray<state_dim> array1(nEl);

    for (size_t i = 0; i < nEl; i++)
    {
        array1[i].setRandom();
    }

    //! create backups for for later comparison
    StateVectorArray<state_dim> array1_backup = array1;

    double scalar = 3.141;

    //! test the overloaded operator
    StateVectorArray<state_dim> result = array1 * scalar;

    for (size_t i = 0; i < nEl; i++)
    {
        //! check that the summation is correct
        ASSERT_EQ(result[i], (array1_backup[i] * scalar));

        //! check that original rhs elements were not altered
        ASSERT_EQ(array1[i], array1_backup[i]);
    }
}


TEST(DiscreteArrayTest, DivisionTest)
{
    const size_t nEl = 10;
    const size_t state_dim = 2;

    StateVectorArray<state_dim> array1(nEl);

    for (size_t i = 0; i < nEl; i++)
    {
        array1[i].setRandom();
    }

    //! create backups for for later comparison
    StateVectorArray<state_dim> array1_backup = array1;

    double scalar = 3.141;

    //! test the overloaded operator
    StateVectorArray<state_dim> result = array1 / scalar;

    for (size_t i = 0; i < nEl; i++)
    {
        //! check that the summation is correct
        ASSERT_EQ(result[i], (array1_backup[i] / scalar));

        //! check that original rhs elements were not altered
        ASSERT_EQ(array1[i], array1_backup[i]);
    }
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
