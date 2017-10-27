/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/core/core.h>
#include "HyALinearizedForward-impl.h"

namespace ct {
namespace models {
namespace HyA {
namespace tpl {

template class HyALinearizedForward<float>;
template class HyALinearizedForward<double>;
// template class HyALinearizedForward<ct::core::ADCGScalar>;
}
}
}
}