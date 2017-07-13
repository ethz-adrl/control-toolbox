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

#ifndef INCLUDE_CT_OPTCON_SOLVER_GNMS_CT_H_
#define INCLUDE_CT_OPTCON_SOLVER_GNMS_CT_H_

#include <ct/optcon/solver/NLOptConSettings.hpp>
#include <ct/optcon/nloc/NLOCAlgorithm.hpp>

namespace ct{
namespace optcon{


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class GNMS_CT : public NLOCAlgorithm<STATE_DIM, CONTROL_DIM, SCALAR>
{

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const size_t STATE_D = STATE_DIM;
	static const size_t CONTROL_D = CONTROL_DIM;

	typedef ct::core::ConstantTrajectoryController<STATE_DIM, CONTROL_DIM, SCALAR> Policy_t;
	typedef NLOptConSettings Settings_t;
	typedef SCALAR Scalar_t;

	typedef NLOCAlgorithm<STATE_DIM, CONTROL_DIM, SCALAR> BASE;
	typedef NLOCBackendBase<STATE_DIM, CONTROL_DIM> Backend_t;

	GNMS_CT(std::shared_ptr<Backend_t>& backend_, const Settings_t& settings) :
		BASE(backend_)
	{
		configure(settings); // todo: might be redundant????
	}

	virtual ~GNMS_CT(){}

	virtual void configure(const Settings_t& settings) override {
		throw(std::runtime_error("to be filled"));
	}

	virtual void prepareIteration() override {
		throw(std::runtime_error("to be filled"));
	}

	virtual bool finishIteration() override {
		throw(std::runtime_error("to be filled"));
		return true;}

	virtual bool runIteration() override {
		throw(std::runtime_error("to be filled"));
		return true;}

	virtual void setInitialGuess(const Policy_t& initialGuess) override {
		throw(std::runtime_error("to be filled"));
	}

};

}
}

#endif /* INCLUDE_CT_OPTCON_SOLVER_GNMS_CT_H_ */
