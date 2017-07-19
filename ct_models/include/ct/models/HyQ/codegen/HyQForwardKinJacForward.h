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

#ifndef INCLUDE_HyQForwardKinJacForward_H_
#define INCLUDE_HyQForwardKinJacForward_H_

#include <ct/core/math/Derivatives.h>

namespace ct {
namespace models {
namespace HyQ {

class HyQForwardKinJacForward : public core::Derivatives<36, 24, double> {
public:
	typedef Eigen::Matrix<double, 24, 36> JAC_TYPE;
	typedef Eigen::Matrix<double, 36, 1> X_TYPE;

	HyQForwardKinJacForward() {
		jac_.setZero();
		v_.fill(0.0);
	};

	HyQForwardKinJacForward(const HyQForwardKinJacForward& other)
	{
		jac_.setZero();
		v_.fill(0.0);
	}

	virtual ~HyQForwardKinJacForward() {};

	HyQForwardKinJacForward* clone() const override{
		return new HyQForwardKinJacForward(*this);
	}


	JAC_TYPE jacobian(const Eigen::VectorXd& x_in) override;

private:
	JAC_TYPE jac_;
	std::array<double, 442> v_;
};

} /* namespace HyQ */
} /* namespace models */
} /* namespace ct */

#endif /* INCLUDE_JACOBIAN_NAME_H_ */
