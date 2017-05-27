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

#ifndef TESTNONLINEARSYSTEM_H_
#define TESTNONLINEARSYSTEM_H_

#include <cmath>
#include <memory>
#include <iostream>

#include <ct/core/core.h>

namespace ct {
namespace core {

namespace tpl
{

template <typename SCALAR>
class TestNonlinearSystem : public ControlledSystem<2, 1, SCALAR>
{
public:

	static const size_t STATE_DIM = 2;
	static const size_t CONTROL_DIM = 1;

	TestNonlinearSystem() = delete;

	// constructor directly using frequency and damping coefficients
	TestNonlinearSystem(SCALAR w_n, std::shared_ptr<Controller<2,1,SCALAR> > controller = nullptr) :
		ControlledSystem<2,1,SCALAR>(controller, SYSTEM_TYPE::GENERAL),
		w_n_(w_n)
	{}

	TestNonlinearSystem(const TestNonlinearSystem& arg) :
	    ControlledSystem<2,1,SCALAR>(arg),
		w_n_(arg.w_n_)
	{}

	TestNonlinearSystem* clone() const override
	{
		return new TestNonlinearSystem(*this);
	}


	void computeControlledDynamics(
			const StateVector<2, SCALAR>& state,
			const SCALAR& t,
			const ControlVector<1, SCALAR>& control,
			StateVector<2, SCALAR>& derivative
	) override
	{
		//this is pretty much random
		derivative(0) = state(1) * state(0) + state(1) * control(0);
		derivative(1) = w_n_ * control(0) - 2.0 * w_n_ * state(1) - 3.0 * w_n_ * state(1) * control(0);
	}

private:
	SCALAR w_n_;
};

}

typedef tpl::TestNonlinearSystem<double> TestNonlinearSystem;

} // namespace core
} // namespace ct

#endif /* TESTNONLINEARSYSTEM_H_ */
