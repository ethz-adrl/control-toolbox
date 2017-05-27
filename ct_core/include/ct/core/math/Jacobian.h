/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#ifndef INCLUDE_CT_CORE_MATH_JACOBIAN_H_
#define INCLUDE_CT_CORE_MATH_JACOBIAN_H_

#include <Eigen/Core>
#include <Eigen/StdVector>

namespace ct {
namespace core {

//! General interface class for a Jacobian
/*!
 * Interface for a general Jacobian of a vector-valued function. Can be used either
 * with fixed size or dynamic size data types
 *
 * @tparam IN_DIM input dimension of function (use Eigen::Dynamic (-1) for dynamic size)
 * @tparam OUT_DIM output dimension of function (use Eigen::Dynamic (-1) for dynamic size)
 * @tparam SCALAR scalar data type
 */
template <int IN_DIM, int OUT_DIM, typename SCALAR = double>
class Jacobian {
public:
	//! The Jacobian data type
	typedef Eigen::Matrix<SCALAR, OUT_DIM, IN_DIM> JAC_TYPE;

	//! The input vector type
	typedef Eigen::Matrix<SCALAR, IN_DIM, 1> X_TYPE;

	//! default constructor
	Jacobian() {};

	//! default destructor
	virtual ~Jacobian() {};

	//! deep copy for derived classes
	virtual Jacobian<IN_DIM, OUT_DIM, SCALAR>* clone() const = 0;

	//! evaluate Jacobian
	/*!
	 * Evaluates the Jacobian at a given state
	 * @param x state at which to evaluate the Jacobian
	 * @return Jacobian matrix
	 */
	virtual JAC_TYPE operator()(const Eigen::VectorXd& x) = 0;
};

} /* namespace core */
} /* namespace ct */

#endif /* INCLUDE_CT_CORE_MATH_JACOBIAN_H_ */
