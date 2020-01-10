/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#ifndef INCLUDE_CT_CORE_CORE_H_
#define INCLUDE_CT_CORE_CORE_H_

#include <iosfwd>
#include <vector>
#include <cstdlib>
#include <functional>

#ifdef CPPADCG
#include <cppad/cg.hpp>
#endif

#ifdef CPPAD
#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>
#include "internal/autodiff/CppadParallel.h"
#endif

// Include file for convenience
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/StdVector>
#include <unsupported/Eigen/MatrixFunctions>


#include "Common"
#include "Types"
#include "Control"
#include "Systems"
#include "Integration"
#include "Geometry"
#include "Internal"
#include "Math"

#include "templateDir.h"

#ifdef PLOTTING_ENABLED
#include "plot/plot.h"
#endif

/*!
 * \warning{do not include implementation files in core-prespec.h}
 */


// keep standard header guard (easy debugging)
// header guard is identical to the one in core.h
#endif  // INCLUDE_CT_CORE_CORE_H_
