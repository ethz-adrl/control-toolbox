/*
 * LQOCSolver.hpp
 *
 *  Created on: Jul 12, 2017
 *      Author: neunertm
 */

#ifndef INCLUDE_CT_OPTCON_LQ_LQOCSOLVER_HPP_
#define INCLUDE_CT_OPTCON_LQ_LQOCSOLVER_HPP_

namespace ct {
namespace optcon {

template <int STATE_DIM, int CONTROL_DIM, typename SCALAR = double>
class LQOCSolver
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR> LQOCProblem;

	LQOCSolver(const std::shared_ptr<LQOCProblem>& lqocProblem = nullptr) :
		lqocProblem_(lqocProblem)
	{
	}

	virtual ~LQOCSolver() {}

	void setProblem(const std::shared_ptr<LQOCProblem>& lqocProblem)
	{
		lqocProblem_ = lqocProblem;
		this->setProblemImpl();
	}

	virtual void configure() = 0;

	virtual void solve() = 0;

	virtual void solveSingleStage(int N) {
		throw std::runtime_error("solveSingleStage not available for this solver.");
	}

	virtual ct::core::StateVectorArray<STATE_DIM, SCALAR> getSolutionState() = 0;
	virtual ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> getSolutionControl() = 0;
	virtual ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR> getFeedback() { throw std::runtime_error("this solver does not provide feedback gains"); }

protected:
	virtual void setProblemImpl(std::shared_ptr<LQOCProblem>& lqocProblem) = 0;

	std::shared_ptr<LQOCProblem> lqocProblem_;

};



}
}



#endif /* INCLUDE_CT_OPTCON_LQ_LQOCSOLVER_HPP_ */
