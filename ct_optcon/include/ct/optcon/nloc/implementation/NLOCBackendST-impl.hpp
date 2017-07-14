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
void NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::createLQProblem()
{
	throw std::runtime_error("should not be used. to be deleted.");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::solveLQProblem()
{
	this->lqocSolver_->setProblem(this->lqocProblem_);
	this->lqocSolver_->solve();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeLinearizedDynamicsAroundTrajectory()
{
	for (size_t k=0; k<this->K_; k++)
	{
		this->computeLinearizedDynamics(this->settings_.nThreads, k);
	}
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeQuadraticCostsAroundTrajectory()
{
	this->initializeCostToGo();

	for (size_t k=0; k<this->K_; k++)
	{
		// compute quadratic cost
		this->computeQuadraticCosts(this->settings_.nThreads, k);
	}
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::updateSolutionState()
{
	this->x_ = this->lqocSolver_->getSolutionState();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::updateSolutionFeedforward()
{
	this->u_ff_ = this->lqocSolver_->getSolutionControl();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::updateSolutionFeedback()
{
	this->L_ = this->lqocSolver_->getFeedback();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::updateShots()
{
	for (size_t k=0; k<this->K_; k++)
	{
//		this->updateSingleShot(this->settings_.nThreads, k);
		this->initializeSingleShot(this->settings_.nThreads, k);
	}
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::initializeShots()
{
	for (size_t k=0; k<this->K_; k++)
	{
		this->initializeSingleShot(this->settings_.nThreads, k);
	}
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeDefects()
{
	this->d_norm_ = 0.0;

	for (size_t k=0; k<this->K_+1; k++)
	{
		this->computeSingleDefect(this->settings_.nThreads, k);
		this->d_norm_ += this->d_[k].norm();
	}
}


} // namespace optcon
} // namespace ct

