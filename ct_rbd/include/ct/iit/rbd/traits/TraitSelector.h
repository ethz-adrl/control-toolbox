/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "FloatTrait.h"
#include "DoubleTrait.h"

namespace iit {
namespace rbd {
namespace tpl {

template <typename SCALAR>
struct TraitSelector
{

};

template <>
struct TraitSelector<double>
{
 	typedef DoubleTrait Trait;
};

template <>
struct TraitSelector<float>
{
 	typedef FloatTrait Trait;
};

} //namespace tpl
} // namespace rbd
} // namespace iit

