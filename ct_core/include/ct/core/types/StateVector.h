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


#ifndef CT_CORE_STATEVECTOR_H_
#define CT_CORE_STATEVECTOR_H_

namespace ct {
namespace core {

template <size_t STATE_DIM, class SCALAR = double>
class StateVector : public Eigen::Matrix<SCALAR, STATE_DIM, 1>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const size_t DIM = STATE_DIM;

	StateVector() {};
	virtual ~StateVector() {};

    typedef Eigen::Matrix<SCALAR, STATE_DIM, 1> Base;
    // This constructor allows you to construct MyVectorType from Eigen expressions
    template<typename OtherDerived>
    StateVector(const Eigen::MatrixBase<OtherDerived>& other)
        : Base(other)
    { }
    // This method allows you to assign Eigen expressions to MyVectorType
    template<typename OtherDerived>
    StateVector& operator=(const Eigen::MatrixBase<OtherDerived>& other)
    {
        this->Base::operator=(other);
        return *this;
    }
};

} /* namespace core */
} /* namespace ct */

#endif /* CT_CORE_STATEVECTOR_H_ */
