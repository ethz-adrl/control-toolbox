/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "DiscreteArray.h"

namespace ct {
namespace core {

/**
 * @brief this method transposes/flips an array of vectors into another array containing the scalar trajectories 
 * corresponding to each dimension of the input array.
 * 
 * Example: if a StateVectorArray<3> with a total length of 100 is the input, the output is a vector of lengt 3, 
 * which contains Eigen-Vectors of length 100 corresponding to the scalar sequence in each dimension of the input array.
 * 
 * @tparam EIGEN_VECTOR_ARRAY_TYPE_IN type of the input array
 * @tparam SCALAR primitive type 
 * @param array the input array to be transposed
 * @return DiscreteArray<Eigen::Matrix<SCALAR, -1, -1>> the output (transposed) array. 
 * 
 * @warning this method is inefficient, to not call at in performance-critical code.
 */
template <typename EIGEN_VECTOR_ARRAY_TYPE_IN, typename SCALAR = double>
DiscreteArray<Eigen::Matrix<SCALAR, -1, -1>> transposeArray(const EIGEN_VECTOR_ARRAY_TYPE_IN& array)
{
    // some runtime tests
    if (array.front().cols() != 1)
    {
        throw std::runtime_error(
            "ct::core::transposeArray() called with invalid type. Eigen types in array must be vectors (no matrices "
            "allowed).");
    }

    size_t rows_in = array.front().rows();
    size_t cols_in = array.size();

    DiscreteArray<Eigen::Matrix<SCALAR, -1, -1>> result;

    for (size_t j = 0; j < rows_in; j++)
    {
        Eigen::Matrix<SCALAR, -1, -1> newVec(cols_in, 1);

        for (size_t i = 0; i < cols_in; i++)
        {
            newVec(i, 0) = array[i](j);
        }
        result.push_back(newVec);
    }
    return result;
}

} /* namespace core */
} /* namespace ct */