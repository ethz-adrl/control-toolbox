#pragma once

#include "EuclideanState.h"

namespace ct {
namespace core {

template <typename T>
struct is_euclidean
{
    static constexpr bool value = false;
};

template <size_t STATE_DIM, typename SCALAR>
struct is_euclidean<EuclideanState<STATE_DIM, SCALAR>>
{
    static constexpr bool value{true};
};

template <typename T>
struct is_real_euclidean
{
    static constexpr bool value{false};
};

template <size_t STATE_DIM>
struct is_real_euclidean<EuclideanState<STATE_DIM, double>>
{
    static constexpr bool value{true};
};

template <size_t STATE_DIM>
struct is_real_euclidean<EuclideanState<STATE_DIM, float>>
{
    static constexpr bool value{true};
};

}  // namespace core
}  // namespace ct
