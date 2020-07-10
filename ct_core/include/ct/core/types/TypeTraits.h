/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "EuclideanState.h"
#include "EuclideanStateSymplectic.h"

namespace ct {
namespace core {

template <typename T>
struct is_euclidean
{
    static constexpr bool value = false;
};

template <int STATE_DIM, typename SCALAR>
struct is_euclidean<EuclideanState<STATE_DIM, SCALAR>>
{
    static constexpr bool value{true};
};

template <int P, int V, typename SCALAR>
struct is_euclidean<EuclideanStateSymplectic<P, V, SCALAR>>
{
    static constexpr bool value{true};
};

template <typename T>
struct is_real_euclidean
{
    static constexpr bool value{false};
};

template <int STATE_DIM>
struct is_real_euclidean<EuclideanState<STATE_DIM, double>>
{
    static constexpr bool value{true};
};

template <int STATE_DIM>
struct is_real_euclidean<EuclideanState<STATE_DIM, float>>
{
    static constexpr bool value{true};
};

template <int P, int V>
struct is_real_euclidean<EuclideanStateSymplectic<P, V, double>>
{
    static constexpr bool value{true};
};

template <int P, int V>
struct is_real_euclidean<EuclideanStateSymplectic<P, V, float>>
{
    static constexpr bool value{true};
};

template <typename T>
struct is_symplectic
{
    static constexpr bool value = false;
};

template <int P, int V, typename SCALAR>
struct is_symplectic<EuclideanStateSymplectic<P, V, SCALAR>>
{
    static constexpr bool value = true;
};

}  // namespace core
}  // namespace ct
