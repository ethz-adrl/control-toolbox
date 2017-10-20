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

#include "ct/optcon/dms/dms_core/spline/SplinerBase.h"
#include <ct/optcon/dms/dms_core/TimeGrid.h>

namespace ct {
namespace optcon {

/**
 * @ingroup    DMS
 *
 * @brief      The spline implementation for the zero order hold spliner
 *
 * @tparam     T     The vector type to be splined
 */
template <class T, typename SCALAR = double>
class ZeroOrderHoldSpliner : public SplinerBase<T, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef T vector_t;
	typedef Eigen::Matrix<SCALAR, T::DIM, T::DIM> matrix_t;
	typedef std::vector<vector_t, Eigen::aligned_allocator<vector_t>> vector_array_t;

	ZeroOrderHoldSpliner() = delete;

	/**
	 * @brief      Custom constructor
	 *
	 * @param[in]  grid  The DMS timegrid
	 */
	ZeroOrderHoldSpliner(std::shared_ptr<tpl::TimeGrid<SCALAR>> grid) : timeGrid_(grid) {}
	virtual ~ZeroOrderHoldSpliner() {}
	void computeSpline(const vector_array_t& points) override { zOholds_ = points; }
	// evaluate spline and return vector at interpolation time
	virtual vector_t evalSpline(const SCALAR time, const size_t shotIdx) override
	{
		assert(shotIdx < zOholds_.size());
		assert(zOholds_[shotIdx] == zOholds_[shotIdx]);
		return zOholds_[shotIdx];
	}

	virtual vector_t splineDerivative_t(const SCALAR time, const size_t shotIdx) const override
	{
		return vector_t::Zero();
	}

	virtual vector_t splineDerivative_h_i(const SCALAR time, const size_t shotIdx) const override
	{
		return vector_t::Zero();
	}

	virtual matrix_t splineDerivative_q_i(const SCALAR time, const size_t shotIdx) const override
	{
		return matrix_t::Identity();
	}

	virtual matrix_t splineDerivative_q_iplus1(const SCALAR time, const size_t shotIdx) const override
	{
		return matrix_t::Zero();
	}


private:
	// zero order hold variables
	vector_array_t zOholds_;

	std::shared_ptr<tpl::TimeGrid<SCALAR>> timeGrid_;
};

}  // namespace optcon
}  // namespace ct
