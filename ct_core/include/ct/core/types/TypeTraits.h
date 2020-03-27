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

template <size_t STATE_DIM, typename SCALAR>
struct is_euclidean<EuclideanState<STATE_DIM, SCALAR>>
{
    static constexpr bool value{true};
};

template <size_t P, size_t V, typename SCALAR>
struct is_euclidean<EuclideanStateSymplectic<P, V, SCALAR>>
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

template <size_t P, size_t V>
struct is_real_euclidean<EuclideanStateSymplectic<P, V, double>>
{
    static constexpr bool value{true};
};

template <size_t P, size_t V>
struct is_real_euclidean<EuclideanStateSymplectic<P, V, float>>
{
    static constexpr bool value{true};
};

template <typename T>
struct is_symplectic
{
    static constexpr bool value = false;
};

template <size_t P, size_t V, typename SCALAR>
struct is_symplectic<EuclideanStateSymplectic<P, V, SCALAR>>
{
    static constexpr bool value = true;
};

}  // namespace core
}  // namespace ct
