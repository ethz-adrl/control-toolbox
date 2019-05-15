/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
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
