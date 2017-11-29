/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "CppADCodegenTrait.h"
#include "CppADDoubleTrait.h"
#include <ct/core/internal/traits/TraitSelector.h>

namespace ct {
namespace core {
namespace tpl {

template <>
struct TraitSelector<CppAD::AD<double>>
{
    typedef internal::CppADDoubleTrait Trait;
};

template <>
struct TraitSelector<CppAD::AD<CppAD::cg::CG<double>>>
{
    typedef internal::CppADCodegenTrait Trait;
};


}  // namespace tpl
}  // namespace core
}  // namespace ct
