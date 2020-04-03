
/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <cppad/cg.hpp>
#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>

namespace ct {
namespace core {

// default out type trait
template <typename T>
struct get_out_type
{
    //static_assert(false, "no out-type trait defined for your type, see EvaluatorOutTypeTraits.h.");
    using type = double;
};

// trivial instance - return type double for double
template <>
struct get_out_type<double>
{
    using type = double;
};

// trivial instance- return type float for float
template <>
struct get_out_type<float>
{
    using type = float;
};

// Cppad instance - return float or double for CppAD::AD<float> and CppAD::AD<double>
template <typename AD_PRIMITIVE>
struct get_out_type<CppAD::AD<AD_PRIMITIVE>>
{
    using type = AD_PRIMITIVE;
};

// Cppad-CG instance - return float or double for CppAD::AD<float> and CppAD::AD<double>
template <typename ADCG_PRIMITIVE>
struct get_out_type<CppAD::AD<CppAD::cg::CG<ADCG_PRIMITIVE>>>
{
    using type = ADCG_PRIMITIVE;
};
}  // namespace core
}  // namespace ct