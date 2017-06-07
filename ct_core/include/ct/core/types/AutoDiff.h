/*
 * AutoDiff.hpp
 *
 *  Created on: Jun 5, 2017
 *      Author: neunertm
 */

#ifndef INCLUDE_CT_CORE_TYPES_AUTODIFF_H_
#define INCLUDE_CT_CORE_TYPES_AUTODIFF_H_

namespace ct {
namespace core {

typedef CppAD::AD<double> ADScalar;
typedef CppAD::cg::CG<double> ADCGValueType;

typedef CppAD::AD<ADCGValueType> ADCGScalar; //!< scalar  type

}
}



#endif /* INCLUDE_CT_CORE_TYPES_AUTODIFF_H_ */
