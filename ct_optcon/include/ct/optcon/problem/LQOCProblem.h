/*
 * LQOptConProblem.h
 *
 *  Created on: Jul 12, 2017
 *      Author: neunertm
 */

#ifndef INCLUDE_CT_OPTCON_PROBLEM_LQOCPROBLEM_H_
#define INCLUDE_CT_OPTCON_PROBLEM_LQOCPROBLEM_H_

namespace ct {
namespace optcon {

template <int STATE_DIM, int CONTROL_DIM, typename SCALAR = double>
class LQOCProblem
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	LQOCProblem(int N = 0) {
		changeNumStages(N);
	}

	int getNumStages()
	{
		return K_;
	}

	void changeNumStages(int N)
	{
		K_ = N;

		A_.resize(N);
		B_.resize(N);

		x_.resize(N+1);
		u_.resize(N);

		d_.resize(N);
		P_.resize(N);
		q_.resize(N+1);
		qv_.resize(N+1);
		Q_.resize(N+1);

		rv_.resize(N);
		R_.resize(N);
	}


	ct::core::StateMatrixArray<STATE_DIM, SCALAR> A_;
	ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR> B_;

	ct::core::StateVectorArray<STATE_DIM, SCALAR> x_;
	ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> u_;

	ct::core::StateVectorArray<STATE_DIM, SCALAR> d_; // the defects

	ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR> P_;

	ct::core::ScalarArray<SCALAR> q_;
	ct::core::StateVectorArray<STATE_DIM, SCALAR> qv_;
	ct::core::StateMatrixArray<STATE_DIM, SCALAR> Q_;

	ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> rv_;
	ct::core::ControlMatrixArray<CONTROL_DIM, SCALAR> R_;

private:
	int K_;

};

}
}



#endif /* INCLUDE_CT_OPTCON_PROBLEM_LQOCPROBLEM_H_ */
