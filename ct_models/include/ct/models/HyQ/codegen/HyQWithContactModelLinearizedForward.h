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

#ifndef CT_HyQWithContactModelLinearizedForward_H_
#define CT_HyQWithContactModelLinearizedForward_H_

#include <ct/core/core.h>

namespace ct {
namespace models {
namespace HyQ {

class HyQWithContactModelLinearizedForward : public ct::core::LinearSystem<36, 12>{

public:

	typedef typename Eigen::Matrix<double, 36, 36> state_matrix_t;
	typedef typename Eigen::Matrix<double, 36, 12> state_control_matrix_t;

	HyQWithContactModelLinearizedForward(const ct::core::SYSTEM_TYPE& type = ct::core::SYSTEM_TYPE::GENERAL):
		ct::core::LinearSystem<36, 12>(type)
	{
		initialize();
	}

	HyQWithContactModelLinearizedForward(const HyQWithContactModelLinearizedForward& other)
	{
		initialize();
	}

	virtual ~HyQWithContactModelLinearizedForward(){};

	virtual HyQWithContactModelLinearizedForward* clone() const override
	{
		return new HyQWithContactModelLinearizedForward;
	}

	virtual const state_matrix_t& getDerivativeState(const ct::core::StateVector<36>& x, const ct::core::ControlVector<12>& u, const double t = 0.0) override;

	virtual const state_control_matrix_t& getDerivativeControl(const ct::core::StateVector<36>& x, const ct::core::ControlVector<12>& u, const double t = 0.0) override;

private:
	void initialize() {
		dFdx_.setZero();
		dFdu_.setZero();
		vX_.fill(0.0);
		vU_.fill(0.0);
	}

	state_matrix_t dFdx_;
	state_control_matrix_t dFdu_;
	std::array<double, 1686> vX_;
	std::array<double, 240> vU_;

};

}
}
}

#endif



