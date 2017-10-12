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

#include "SISOControllerBase.h"

namespace ct {
namespace core {

//! A simple step input
/*!
 * A step input controller with a standard heaviside form
 *
 * \f[
 *
 *
 * \begin{aligned}
 * u(t) \mapsto
 * \begin{cases}
 * 0 : & t < t_{step} \\
 * g : & t \ge t_{step}
 * \end{cases}
 * \end{aligned}
 *
 * \f]
 *
 * where \f$ g \f$ is a constant gain and \f$ t_{step} \f$ is the time of the step.
 */
class StepInputController : public SISOControllerBase
{
public:
	//! Parameters of the step input function
	/*!
	 * Contains the constant gain \f$ g \f$ and the time of the step \f$ t_{step} \f$
	 */
	struct Parameters {
		Parameters(double gain_ = 1.0, double t_step_ = 1.0) :
			gain(gain_),
			t_step(t_step_)
		{}

		double gain; //! gain
		Time t_step; //! time of step
	};

	//! default constructor
	StepInputController(const Parameters& parameters = Parameters()) :
	    parameters_(parameters)
	{}

	//! copy constructor
	StepInputController(const StepInputController& arg):
		parameters_(arg.parameters_)
	{}

	//! deep cloning
	StepInputController* clone() const {return new StepInputController(*this);}

	//! computes control input
	/*!
	 * Computes the control input. The state parameter gets ignored.
	 * @param state current state (ignored)
	 * @param t current time
	 * @return control action, either 0 or g
	 */
	double computeControl(const double& state, const Time& t) override
	{
		return parameters_.gain*(t >= parameters_.t_step);
	}

private:
	Parameters parameters_; //! parameters of the step function

};


}
}

