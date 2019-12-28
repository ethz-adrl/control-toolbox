/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#ifdef CPPADCG
#include <cppad/cg.hpp>
#endif

#ifdef CPPAD
#include <cppad/cppad.hpp>
#endif

namespace ct {
namespace core {

#ifdef CPPAD
typedef CppAD::AD<double> ADScalar;
#endif

#ifdef CPPADCG
typedef CppAD::cg::CG<double> ADCGValueType;
typedef CppAD::AD<ADCGValueType> ADCGScalar;  //!< scalar  type
#endif
}
}
