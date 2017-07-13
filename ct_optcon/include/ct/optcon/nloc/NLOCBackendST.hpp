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

namespace ct{
namespace optcon{


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM = STATE_DIM/2, size_t V_DIM=STATE_DIM/2, typename SCALAR = double>
class NLOCBackendST : public NLOCBackendBase <STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR> Base;

	NLOCBackendST(const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem,
			const GNMSSettings& settings) :
				Base(optConProblem, settings)
	{}

	NLOCBackendST(const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem,
			 const std::string& settingsFile,
			 bool verbose = true,
			 const std::string& ns = "ilqg") :
			Base(optConProblem, settingsFile, verbose, ns)
	{}

	virtual ~NLOCBackendST() {};

protected:

	virtual void createLQProblem() override;

	virtual void solveLQProblem() override;

	virtual void computeLinearizedDynamicsAroundTrajectory() override;

	virtual void computeQuadraticCostsAroundTrajectory() override;

	virtual void updateControlAndState() override;

	virtual void updateShots() override;

	virtual void initializeShots() override;

	virtual void computeDefects() override;
};


} // namespace optcon
} // namespace ct

#include "implementation/NLOCBackendST-impl.hpp"


#endif /* INCLUDE_CT_OPTCON_NLOC_BACKEND_ST_HPP_ */
