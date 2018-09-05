/*!
 * a convenience include file collecting all class declarations related to cost functions
 */

#pragma once

// time activations
#include "term/TermBase.hpp"
#include "term/TermLinear.hpp"
#include "term/TermMixed.hpp"
#include "term/TermQuadratic.hpp"
#include "term/TermQuadMult.hpp"
#include "term/TermQuadTracking.hpp"
#include "term/TermStateBarrier.hpp"
#include "term/TermSmoothAbs.hpp"

// costfunctions
#include "CostFunction.hpp"
#include "CostFunctionQuadratic.hpp"
#include "CostFunctionAD.hpp"
#include "CostFunctionAnalytical.hpp"
#include "CostFunctionQuadraticSimple.hpp"
