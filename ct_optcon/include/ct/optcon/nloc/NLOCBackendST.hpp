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


#ifndef INCLUDE_CT_OPTCON_NLOC_BACKEND_ST_HPP_
#define INCLUDE_CT_OPTCON_NLOC_BACKEND_ST_HPP_

#include "NLOCBackendBase.hpp"
#include "NLOCBackendBase-impl.hpp"
#include <ct/optcon/solver/NLOptConSettings.hpp>

namespace ct{
namespace optcon{


/*!
 * NLOC Backend for Single-Threaded case
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR = double>
class NLOCBackendST : public NLOCBackendBase <STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR> Base;

	NLOCBackendST(const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem,
			const NLOptConSettings& settings);

	NLOCBackendST(const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem,
			 const std::string& settingsFile,
			 bool verbose = true,
			 const std::string& ns = "alg");

	virtual ~NLOCBackendST();

protected:

	virtual void computeLinearizedDynamicsAroundTrajectory(size_t firstIndex, size_t lastIndex) override;

	virtual void computeQuadraticCostsAroundTrajectory(size_t firstIndex, size_t lastIndex) override;

	virtual void rolloutShots(size_t firstIndex, size_t lastIndex) override;

	SCALAR performLineSearch() override;
};


} // namespace optcon
} // namespace ct


#endif /* INCLUDE_CT_OPTCON_NLOC_BACKEND_ST_HPP_ */
