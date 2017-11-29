/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

typedef CppAD::AD<double> ADScalar;
typedef CppAD::cg::CG<double> ADCGValueType;

typedef CppAD::AD<ADCGValueType> ADCGScalar;  //!< scalar  type
}
}
