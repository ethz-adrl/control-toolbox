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


namespace ct{
namespace optcon{


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR = double>
class NLOCAlgorithm
{

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR> Backend_t;

	typedef ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR> Policy_t;
	typedef NLOptConSettings Settings_t;
	typedef SCALAR Scalar_t;

	NLOCAlgorithm(const std::shared_ptr<Backend_t>& backend) :
		backend_(backend)
	{}

	virtual ~NLOCAlgorithm(){}

	virtual void configure(const Settings_t& settings)  = 0;

	virtual void prepareIteration()  = 0;

	virtual bool finishIteration() = 0;

	virtual bool runIteration() = 0;

	virtual void setInitialGuess(const Policy_t& initialGuess) = 0;

	virtual void prepareMPCIteration()  = 0;

	virtual bool finishMPCIteration() = 0;

protected:
	std::shared_ptr<Backend_t> backend_;

};

}
}



