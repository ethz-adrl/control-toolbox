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

#pragma once

namespace ct {
namespace optcon {

/**
 * @ingroup    DMS
 *
 * @brief      Abstract base class for the control input splining between the
 *             DMS shots
 *
 * @tparam     T     The vector type which will be splined
 */
template <class T, typename SCALAR = double>
class SplinerBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 * @brief      Default constructor
	 */
	SplinerBase(){};

	/**
	 * @brief      Destructor
	 */
	virtual ~SplinerBase(){};

	typedef T vector_t;
	typedef Eigen::Matrix<SCALAR, T::DIM, T::DIM> matrix_t;
	typedef std::vector<vector_t, Eigen::aligned_allocator<vector_t>> vector_array_t;


	/**
	 * @brief      Updates the vector on the shots
	 *
	 * @param[in]  points  Updated vector array
	 */
	virtual void computeSpline(const vector_array_t& points) = 0;

	/**
	 * @brief      Depending on the spline type, this method evaluates the
	 *             control input between the shots
	 *
	 * @param[in]  time     The evaluation time
	 * @param[in]  shotIdx  The shot number
	 *
	 * @return     The splined vector
	 */
	virtual vector_t evalSpline(const SCALAR time, const size_t shotIdx) = 0;

	/**
	 * @brief      Returns the spline derivatives with respect to time
	 *
	 * @param[in]  time     The evaluation time
	 * @param[in]  shotIdx  The shot number
	 *
	 * @return     The time derivative
	 */
	virtual vector_t splineDerivative_t(const SCALAR time, const size_t shotIdx) const = 0;

	/**
	 * @brief      Returns the spline derivatives with respect to the time
	 *             segment between the shots
	 *
	 * @param[in]  time     The evaluation time
	 * @param[in]  shotIdx  The shot number
	 *
	 * @return     The resulting derivative
	 */
	virtual vector_t splineDerivative_h_i(const SCALAR time, const size_t shotIdx) const = 0;

	/**
	 * @brief      Return the spline derivative with respect to the control
	 *             input at shot i
	 *
	 * @param[in]  time     The evaluation time
	 * @param[in]  shotIdx  The shot number
	 *
	 * @return     The resulting derivative
	 */
	virtual matrix_t splineDerivative_q_i(const SCALAR time, const size_t shotIdx) const = 0;

	/**
	 * @brief      Returns the spline derivative with respect to the control
	 *             input at shot i+1
	 *
	 * @param[in]  time     The evaluation time
	 * @param[in]  shotIdx  The shot number
	 *
	 * @return     The resulting derivative
	 */
	virtual matrix_t splineDerivative_q_iplus1(const SCALAR time, const size_t shotIdx) const = 0;
};

}  // namespace optcon
}  // namespace ct
