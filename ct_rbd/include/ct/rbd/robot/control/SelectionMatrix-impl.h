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

#ifndef INCLUDE_CT_RBD_ROBOT_CONTROL_SELECTIONMATRIX_IMPL_H_
#define INCLUDE_CT_RBD_ROBOT_CONTROL_SELECTIONMATRIX_IMPL_H_

namespace ct {
namespace rbd {

template <size_t CONTROL_DIM, size_t STATE_DIM, typename SCALAR>
SelectionMatrix<CONTROL_DIM, STATE_DIM, SCALAR>::SelectionMatrix(const SelectionMatrix& other) :
Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM>(other)
{}

template <size_t CONTROL_DIM, size_t STATE_DIM, typename SCALAR>
SelectionMatrix<CONTROL_DIM, STATE_DIM, SCALAR>::SelectionMatrix(bool floatingBase) :
Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM>()
{
	setIdentity(floatingBase);
}

template <size_t CONTROL_DIM, size_t STATE_DIM, typename SCALAR>
void SelectionMatrix<CONTROL_DIM, STATE_DIM, SCALAR>::setIdentity(bool floatingBase)
{
	this->setZero();

	if (floatingBase)
	{
		if (STATE_DIM < 6)
			throw std::runtime_error("Selection Matrix for floating base systems should at least have 6 columns");

		this->template bottomRightCorner<CONTROL_DIM, STATE_DIM-6>().setIdentity();

	} else
	{
		this->template bottomRightCorner<CONTROL_DIM, CONTROL_DIM>().setIdentity();
	}
}

}
}

#endif /* INCLUDE_CT_RBD_ROBOT_CONTROL_SELECTIONMATRIX_IMPL_H_ */
