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


namespace ct{
namespace optcon{


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::initialize(const OptConProblem_t& optConProblem, const Settings_t& settings)
{

	if(settings.nThreads > 1)
		nlocBackend_ = std::shared_ptr<NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>>(
				new NLOCBackendMP<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>(optConProblem, settings));
	else
		nlocBackend_ = std::shared_ptr<NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>>(
				new NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>(optConProblem, settings));

	configure(settings);

}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::configure(const Settings_t& settings)
{
	if (nlocBackend_->getSettings().nThreads != settings.nThreads)
		throw std::runtime_error("cannot switch from ST to MT or vice versa. Please call initialize.");

	nlocBackend_->configure(settings);

	switch(settings.nlocp_algorithm)
	{
	case NLOptConSettings::NLOCP_ALGORITHM::GNMS:
		nlocAlgorithm_ = std::shared_ptr<NLOCAlgorithm<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>> (
				new GNMS<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>(nlocBackend_, settings) );
		break;
	case NLOptConSettings::NLOCP_ALGORITHM::ILQR:
		nlocAlgorithm_ = std::shared_ptr<NLOCAlgorithm<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>> (
				new iLQR<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>(nlocBackend_, settings) );
		break;
	default:
		throw std::runtime_error("This algorithm is not implemented in NLOptConSolver.hpp");
	}
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
bool NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::solve()
{
	bool foundBetter = true;
	int numIterations = 0;

	while (foundBetter  && (numIterations < nlocBackend_->getSettings().max_iterations))
	{
		prepareIteration();

		foundBetter = finishIteration();

		numIterations++;
	}

	return (numIterations > 1 || foundBetter || (numIterations == 1 && !foundBetter));
}

} // namespace optcon
} // namespace ct


