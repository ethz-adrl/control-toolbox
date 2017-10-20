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
 * @brief      Defines basic types used in the DMS algorithm
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DmsDimensions
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef ct::core::StateVector<STATE_DIM, SCALAR> state_vector_t;
	typedef ct::core::StateVectorArray<STATE_DIM, SCALAR> state_vector_array_t;

	typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t;
	typedef ct::core::StateMatrixArray<STATE_DIM, SCALAR> state_matrix_array_t;

	typedef Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM> state_control_matrix_t;
	typedef ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR> state_control_matrix_array_t;

	typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> control_state_matrix_t;

	typedef ct::core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;
	typedef ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> control_vector_array_t;

	typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
	typedef ct::core::ControlMatrixTrajectory<CONTROL_DIM, SCALAR> control_matrix_array_t;

	typedef SCALAR time_t;
	typedef ct::core::tpl::TimeArray<SCALAR> time_array_t;
};

}  // namespace optcon
}  // namespace ct
