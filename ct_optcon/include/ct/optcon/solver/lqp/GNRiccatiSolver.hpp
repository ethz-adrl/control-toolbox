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

#ifndef INCLUDE_CT_OPTCON_LQ_GNRICCATISOLVER_HPP_
#define INCLUDE_CT_OPTCON_LQ_GNRICCATISOLVER_HPP_

namespace ct {
namespace optcon {

/*!
 * This class implements an general Riccati backward pass for solving an unconstrained
 *  linear-quadratic Optimal Control problem
 */
template <int STATE_DIM, int CONTROL_DIM, typename SCALAR = double>
class GNRiccatiSolver : public LQOCSolver<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const int state_dim = STATE_DIM;
	static const int control_dim = CONTROL_DIM;

	typedef LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR> LQOCProblem;

	typedef ct::core::StateMatrix<STATE_DIM, SCALAR> StateMatrix;
	typedef ct::core::StateMatrixArray<STATE_DIM, SCALAR> StateMatrixArray;
	typedef ct::core::ControlMatrix<CONTROL_DIM, SCALAR> ControlMatrix;
	typedef ct::core::ControlMatrixArray<CONTROL_DIM, SCALAR> ControlMatrixArray;
	typedef ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR> StateControlMatrixArray;
	typedef ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR> FeedbackArray;

	typedef ct::core::StateVectorArray<STATE_DIM, SCALAR> StateVectorArray;
	typedef ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> ControlVectorArray;

	GNRiccatiSolver(const std::shared_ptr<LQOCProblem>& lqocProblem = nullptr) :
		LQOCSolver<STATE_DIM, CONTROL_DIM, SCALAR>(lqocProblem)
	{}

	virtual ~GNRiccatiSolver() {}

	virtual void solve() override
	{
		smallestEigenvalue_ =  std::numeric_limits<SCALAR>::infinity();

		for (int i=this->lqocProblem_->getNumberOfStages()-1; i>=0; i--)
			solveSingleStage(i);
	}

	virtual void solveSingleStage(int N) override
	{
		if (N == lqocProblem->getNumberOfStages + 1)
			initializeCostToGo();

		designController(N);
		computeCostToGo(N);
	}

	virtual void configure() override
	{
		settings_ = settings;
		H_corrFix_ = settings_.epsilon*ControlMatrix::Identity();
	}

	virtual ct::core::StateVectorArray<STATE_DIM, SCALAR> getSolutionState() override { return lx_; }
	virtual ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> getSolutionControl() override { return lv_; }
	virtual ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR> getFeedback() override { return L_; }

protected:

	virtual void setProblemImpl(std::shared_ptr<LQOCProblem>& lqocProblem) override
	{
		const int& N = lqocProblem->getNumberOfStages();

		H_.resize(N);
		Hi_.resize(N);
		Hi_inverse_.resize(N);
		H_corrFix_.resize(N);

		lv_.resize(N);
		L_.resize(N);

		lx_.resize(N+1); // differential update on the state

		sv_.resize(N+1);
		S_.resize(N+1);
	}

	void initializeCostToGo()
	{
		// initialize quadratic approximation of cost to go
		const int& N = lqocProblem->getNumberOfStages();
		S_[N+1] = p.Q_[N+1];
		sv_[N+1] = p.qv_[N+1];
	}

	void computeCostToGo(size_t k)
	{
		LQOCProblem& p = *this->lqocProblem_;

		S_[k] = p.Q_[k];
		S_[k].noalias() += p.A_[k].transpose() * S_[k+1] * p.A_[k];
		S_[k].noalias() -= L_[k].transpose() * Hi_[k] * L_[k];

		S_[k] = 0.5*(p.S_[k]+p.S_[k].transpose()).eval();

		sv_[k] = p.qv_[k];
		sv_[k].noalias() += p.A_[k].transpose() * sv_[k+1];
		sv_[k].noalias() += p.A_[k].transpose() * S_[k+1] * p.d_[k]; // additional riccati term for lifted GNMS
		sv_[k].noalias() += p.L_[k].transpose() * p.Hi_[k] * p.lv_[k];
		sv_[k].noalias() += p.L_[k].transpose() * p.gv_[k];
		sv_[k].noalias() += p.G_[k].transpose() * p.lv_[k];

	}

	void designController(size_t k)
	{
		LQOCProblem& p = *this->lqocProblem_;

		p.gv_[k] = p.rv_[k];
		p.gv_[k].noalias() += p.B_[k].transpose() * sv_[k+1];
		p.gv_[k].noalias() += p.B_[k].transpose() * S_[k+1].template selfadjointView<Eigen::Lower>() * p.d_[k];

		p.G_[k] = p.P_[k];
		//G_[k].noalias() += B_[k].transpose() * S_[k+1] * A_[k];
		p.G_[k].noalias() += p.B_[k].transpose() * S_[k+1].template selfadjointView<Eigen::Lower>() * p.A_[k];

		p.H_[k] = p.R_[k];
		//H_[k].noalias() += B_[k].transpose() * S_[k+1] * B_[k];
		p.H_[k].noalias() += p.B_[k].transpose() * S_[k+1].template selfadjointView<Eigen::Lower>() * p.B_[k];

		if(settings_.fixedHessianCorrection)
		{
			if (settings_.epsilon > 1e-10)
				Hi_[k] = H_[k] + settings_.epsilon*control_matrix_t::Identity();
			else
				Hi_[k] = H_[k];

			if (settings_.recordSmallestEigenvalue)
			{
				// compute eigenvalues with eigenvectors enabled
				eigenvalueSolver_.compute(Hi_[k], Eigen::ComputeEigenvectors);
				const control_matrix_t& V = eigenvalueSolver_.eigenvectors().real();
				const control_vector_t& lambda = eigenvalueSolver_.eigenvalues();

				smallestEigenvalue_ = std::min(smallestEigenvalue_, lambda.minCoeff());

				// Corrected Eigenvalue Matrix
				control_matrix_t D = control_matrix_t::Zero();
				// make D positive semi-definite (as described in IV. B.)
				D.diagonal() = lambda.cwiseMax(settings_.epsilon);

				// reconstruct H
				control_matrix_t Hi_regular = V * D * V.transpose();

				// invert D
				control_matrix_t D_inverse = control_matrix_t::Zero();
				// eigenvalue-wise inversion
				D_inverse.diagonal() = -1.0 * D.diagonal().cwiseInverse();
				control_matrix_t Hi_inverse_regular = V * D_inverse * V.transpose();

				if (!Hi_inverse_[k].isApprox(Hi_inverse_regular, 1e-4))
				{
					std::cout << "warning, inverses not identical at "<<k<<std::endl;
					std::cout << "Hi_inverse_fixed - Hi_inverse_regular: "<<std::endl<<Hi_inverse_[k]-Hi_inverse_regular<<std::endl<<std::endl;
				}

			}

			Hi_inverse_[k] = -Hi_[k].template selfadjointView<Eigen::Lower>().llt().solve(control_matrix_t::Identity());

			// calculate FB gain update
			L_[k].noalias() = Hi_inverse_[k].template selfadjointView<Eigen::Lower>() * G_[k];

			// calculate FF update
			lv_[k].noalias() = Hi_inverse_[k].template selfadjointView<Eigen::Lower>() * gv_[k];

		} else {

			// compute eigenvalues with eigenvectors enabled
			eigenvalueSolver_.compute(H_[k], Eigen::ComputeEigenvectors);
			const control_matrix_t& V = eigenvalueSolver_.eigenvectors().real();
			const control_vector_t& lambda = eigenvalueSolver_.eigenvalues();

			if (settings_.recordSmallestEigenvalue)
			{
				smallestEigenvalue_ = std::min(smallestEigenvalue_, lambda.minCoeff());
			}

			// Corrected Eigenvalue Matrix
			control_matrix_t D = control_matrix_t::Zero();
			// make D positive semi-definite (as described in IV. B.)
			D.diagonal() = lambda.cwiseMax(settings_.epsilon);

			// reconstruct H
			Hi_[k].noalias() = V * D * V.transpose();

			// invert D
			control_matrix_t D_inverse = control_matrix_t::Zero();
			// eigenvalue-wise inversion
			D_inverse.diagonal() = -1.0 * D.diagonal().cwiseInverse();
			Hi_inverse_[k].noalias() = V * D_inverse * V.transpose();

			// calculate FB gain update
			L_[k].noalias() = Hi_inverse_[k] * G_[k];

			// calculate FF update
			lv_[k].noalias() = Hi_inverse_[k] * gv_[k];
			du_norm_ += lv_[k].norm();
		}
	}

	ControlVectorArray gv_;
	FeedbackArray G_;

	ControlMatrixArray H_;
	ControlMatrixArray Hi_;
	ControlMatrixArray Hi_inverse_;
	ControlMatrix H_corrFix_;

	ControlVectorArray lv_;
	FeedbackArray L_;

	StateVectorArray lx_; // differential update on the state

	StateVectorArray sv_;
	StateMatrixArray S_;

	SCALAR smallestEigenvalue_;

	//! Eigenvalue solver, used for inverting the Hessian and for regularization
	Eigen::SelfAdjointEigenSolver<ControlMatrix> eigenvalueSolver_;
};


}
}

#endif /* INCLUDE_CT_OPTCON_LQ_GNRICCATISOLVER_HPP_ */
