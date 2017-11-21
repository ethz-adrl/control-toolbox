/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "FloatTrait.h"
#include "DoubleTrait.h"

namespace ct {
namespace core {
namespace tpl {

template <typename SCALAR>
struct TraitSelector
{
};

template <>
struct TraitSelector<double>
{
    typedef internal::DoubleTrait Trait;
};

template <>
struct TraitSelector<float>
{
    typedef internal::FloatTrait Trait;
};

}  //namespace tpl
}  // namespace core
}  // namespace ct
