/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#ifdef CPPADCG
#include "CppADCodegenTrait.h"
#endif 
#ifdef CPPAD
#include "CppADDoubleTrait.h"
#endif 
#include <ct/core/internal/traits/TraitSelector.h>

namespace ct {
namespace core {
namespace tpl {

#ifdef CPPAD
template <>
struct TraitSelector<CppAD::AD<double>>
{
    typedef internal::CppADDoubleTrait Trait;
};
#endif

#ifdef CPPADCG
template <>
struct TraitSelector<CppAD::AD<CppAD::cg::CG<double>>>
{
    typedef internal::CppADCodegenTrait Trait;
};
#endif

}  // namespace tpl
}  // namespace core
}  // namespace ct
