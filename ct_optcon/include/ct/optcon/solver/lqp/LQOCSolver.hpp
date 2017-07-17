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

#ifndef INCLUDE_CT_OPTCON_LQ_LQOCSOLVER_HPP_
#define INCLUDE_CT_OPTCON_LQ_LQOCSOLVER_HPP_


#include <ct/optcon/solver/NLOptConSettings.hpp>
#include <ct/optcon/problem/LQOCProblem.hpp>

namespace ct {
namespace optcon {

/*!
 * Base class for solvers to solve an LQOCProblem (both constrained / unconstrained, etc.)
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class LQOCSolver
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR> LQOCProblem_t;

	/*!
	 * Constructor. Initialize by handing over an LQOCProblem, or otherwise by calling setProblem()
	 * @param lqocProblem shared_ptr to the LQOCProblem to be solved.
	 */
	LQOCSolver(const std::shared_ptr<LQOCProblem_t>& lqocProblem = nullptr) :
		lqocProblem_(lqocProblem)
	{}

	//! destructor
	virtual ~LQOCSolver() {}

	/*!
	 * update the shared_ptr to the LQOCProblem instance and call initialize instance deriving from this class.
	 * @param lqocProblem
	 */
	void setProblem(std::shared_ptr<LQOCProblem_t>& lqocProblem)
	{
		lqocProblem_ = lqocProblem;
		setProblemImpl(lqocProblem);
	}

	virtual void configure(const NLOptConSettings& settings) = 0;

	virtual void solve() = 0;

	virtual void solveSingleStage(int N) { throw std::runtime_error("solveSingleStage not available for this solver.");}

	virtual void computeStateAndControlUpdates() = 0;

	virtual ct::core::StateVectorArray<STATE_DIM, SCALAR> getSolutionState() = 0;

	virtual ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> getSolutionControl() = 0;

	virtual ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR> getFeedback() { throw std::runtime_error("this solver does not provide feedback gains"); }

	const SCALAR& getControlUpdateNorm() {return delta_uff_norm_;}

	const SCALAR& getStateUpdateNorm() {return delta_x_norm_;}

	const core::StateVectorArray<STATE_DIM, SCALAR>& getStateUpdates() {return lx_;}

	const core::ControlVectorArray<CONTROL_DIM, SCALAR>& getControlUpdates() {return lu_;}

protected:

	virtual void setProblemImpl(std::shared_ptr<LQOCProblem_t>& lqocProblem) = 0;

	std::shared_ptr<LQOCProblem_t> lqocProblem_;

	core::StateVectorArray<STATE_DIM, SCALAR> lx_; // differential update on the state
	core::ControlVectorArray<CONTROL_DIM, SCALAR> lu_; // differential update on the control

	SCALAR delta_uff_norm_;	//! l2-norm of the control update
	SCALAR delta_x_norm_;	//! l2-norm of the state update

};



}
}



#endif /* INCLUDE_CT_OPTCON_LQ_LQOCSOLVER_HPP_ */
