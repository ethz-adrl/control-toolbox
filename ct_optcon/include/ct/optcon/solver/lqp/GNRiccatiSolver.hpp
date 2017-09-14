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

#include "LQOCSolver.hpp"

#ifdef MATLAB_FULL_LOG
#include <ct/optcon/matlab.hpp>
#endif

namespace ct {
namespace optcon {

/*!
 * This class implements an general Riccati backward pass for solving an unconstrained
 *  linear-quadratic Optimal Control problem
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class GNRiccatiSolver : public LQOCSolver<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const int state_dim = STATE_DIM;
	static const int control_dim = CONTROL_DIM;

	typedef LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR> LQOCProblem_t;

	typedef ct::core::StateMatrix<STATE_DIM, SCALAR> StateMatrix;
	typedef ct::core::StateMatrixArray<STATE_DIM, SCALAR> StateMatrixArray;
	typedef ct::core::ControlVector<CONTROL_DIM, SCALAR> ControlVector;
	typedef ct::core::ControlMatrix<CONTROL_DIM, SCALAR> ControlMatrix;
	typedef ct::core::ControlMatrixArray<CONTROL_DIM, SCALAR> ControlMatrixArray;
	typedef ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR> StateControlMatrixArray;
	typedef ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR> FeedbackArray;

	typedef ct::core::StateVectorArray<STATE_DIM, SCALAR> StateVectorArray;
	typedef ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> ControlVectorArray;

	GNRiccatiSolver(const std::shared_ptr<LQOCProblem_t>& lqocProblem = nullptr) :
		LQOCSolver<STATE_DIM, CONTROL_DIM, SCALAR>(lqocProblem),
		N_(-1)
	{
		Eigen::initParallel();
		Eigen::setNbThreads(settings_.nThreadsEigen);
	}

	GNRiccatiSolver(int N)
	{
		changeNumberOfStages(N);
	}

	virtual ~GNRiccatiSolver() {}


	virtual void solve() override
	{
		for (int i=this->lqocProblem_->getNumberOfStages()-1; i>=0; i--)
			solveSingleStage(i);

		computeStateAndControlUpdates();
	}


	virtual void solveSingleStage(int N) override
	{
		if (N == this->lqocProblem_->getNumberOfStages()-1)
			initializeCostToGo();

		designController(N);

		if (N>0)
			computeCostToGo(N);
	}


	virtual void configure(const NLOptConSettings& settings) override
	{
		settings_ = settings;
		H_corrFix_ = settings_.epsilon*ControlMatrix::Identity();
	}


	virtual ct::core::StateVectorArray<STATE_DIM, SCALAR> getSolutionState() override
	{
		LQOCProblem_t& p = *this->lqocProblem_;
		ct::core::StateVectorArray<STATE_DIM, SCALAR> x = p.x_;

		for(int k = 0; k<this->lqocProblem_->getNumberOfStages() +1 ; k++)
		{
			x[k] += this->lx_[k];

//			std::cout << "A: "<<std::endl<<p.A_[k]<<std::endl<<std::endl;
//			std::cout << "B: "<<std::endl<<p.B_[k]<<std::endl<<std::endl;
//			std::cout << "H: "<<std::endl<<H_[k]<<std::endl<<std::endl;
//			std::cout << "S: "<<std::endl<<S_[k]<<std::endl<<std::endl;
//			std::cout << "sv: "<<std::endl<<sv_[k]<<std::endl<<std::endl;
//			std::cout << "L: "<<std::endl<<L_[k]<<std::endl<<std::endl;
//			std::cout << "lv_: "<<std::endl<<lv_[k].transpose()<<std::endl<<std::endl;
//			std::cout << "lx_: "<<std::endl<<this->lx_[k].transpose()<<std::endl<<std::endl;
//			std::cout << std::endl << std::endl;

		}

		return x;
	}

	virtual ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> getSolutionControl() override
	{
		LQOCProblem_t& p = *this->lqocProblem_;

		ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> u = p.u_;

		for(int k = 0; k<this->lqocProblem_->getNumberOfStages(); k++)
		{
			u[k] += this->lu_[k];
		}
		return u;
	}

	virtual ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> getFeedforwardUpdates() override {return lv_;}

	virtual void getFeedback(ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR>& K) override {	K = L_;	}



	//! compute the state and control updates.
	/*!
	 * this method is specific to the GN Riccati solver, since the state updates lx_
	 * need to be completed in an additional forward sweep.
	 *
	 * IMPORTANT: you need to call this method at the right place if you're using solveSingleStage() by yourself.
	 */
	virtual void computeStateAndControlUpdates() override
	{
		LQOCProblem_t& p = *this->lqocProblem_;

		this->delta_x_norm_ = 0.0;
		this->delta_uff_norm_ = 0.0;

		this->lx_[0].setZero();

		for(int k = 0; k < this->lqocProblem_->getNumberOfStages(); k++)
		{
			//! control update rule
			this->lu_[k] = lv_[k];
			if (k>0) // lx is zero for k=0
				this->lu_[k].noalias() += L_[k] * this->lx_[k];

			//! state update rule
			this->lx_[k+1] = p.B_[k] * lv_[k] + p.b_[k];
			if(k>0)
				this->lx_[k+1].noalias() += (p.A_[k] + p.B_[k] * L_[k]) * this->lx_[k];

			//! compute the norms of the updates
			//! \todo needed?
			this->delta_x_norm_ += this->lx_[k+1].norm();
			this->delta_uff_norm_ += this->lu_[k].norm();
		}
	}

	virtual SCALAR getSmallestEigenvalue() override {return smallestEigenvalue_;}

protected:

	/*!
	 * resize matrices
	 * @param lqocProblem
	 */
	virtual void setProblemImpl(std::shared_ptr<LQOCProblem_t> lqocProblem) override
	{
		const int& N = lqocProblem->getNumberOfStages();
		changeNumberOfStages(N);

	}
	void changeNumberOfStages(int N)
	{
		if (N<=0)
			return;

		if(N_ == N)
	 		return;

		gv_.resize(N);
		G_.resize(N);

		H_.resize(N);
		Hi_.resize(N);
		Hi_inverse_.resize(N);

		lv_.resize(N);
		L_.resize(N);

		this->lx_.resize(N+1);
		this->lu_.resize(N);

		sv_.resize(N+1);
		S_.resize(N+1);

		N_ = N;
	}


	void initializeCostToGo()
	{
		//! since intializeCostToGo is the first call, we initialize the smallestEigenvalue here.
		smallestEigenvalue_ =  std::numeric_limits<SCALAR>::infinity();

		// initialize quadratic approximation of cost to go
		const int& N = this->lqocProblem_->getNumberOfStages();
		LQOCProblem_t& p = *this->lqocProblem_;

		S_[N] = p.Q_[N];
		sv_[N] = p.qv_[N];
	}

	void computeCostToGo(size_t k)
	{
		LQOCProblem_t& p = *this->lqocProblem_;

		S_[k] = p.Q_[k];
		S_[k].noalias() += p.A_[k].transpose() * S_[k+1] * p.A_[k];
		S_[k].noalias() -= L_[k].transpose() * Hi_[k] * L_[k];

		S_[k] = 0.5*(S_[k]+S_[k].transpose()).eval();

		sv_[k] = p.qv_[k];
		sv_[k].noalias() += p.A_[k].transpose() * sv_[k+1];
		sv_[k].noalias() += p.A_[k].transpose() * S_[k+1] * p.b_[k];
		sv_[k].noalias() += L_[k].transpose() * Hi_[k] * lv_[k];
		sv_[k].noalias() += L_[k].transpose() * gv_[k];
		sv_[k].noalias() += G_[k].transpose() * lv_[k];
	}

	void designController(size_t k)
	{
		LQOCProblem_t& p = *this->lqocProblem_;

		gv_[k] = p.rv_[k];
		gv_[k].noalias() += p.B_[k].transpose() * sv_[k+1];
		gv_[k].noalias() += p.B_[k].transpose() * S_[k+1].template selfadjointView<Eigen::Lower>() * p.b_[k];

		G_[k] = p.P_[k];
		//G_[k].noalias() += B_[k].transpose() * S_[k+1] * A_[k];
		G_[k].noalias() += p.B_[k].transpose() * S_[k+1].template selfadjointView<Eigen::Lower>() * p.A_[k];

		H_[k] = p.R_[k];
		//H_[k].noalias() += B_[k].transpose() * S_[k+1] * B_[k];
		H_[k].noalias() += p.B_[k].transpose() * S_[k+1].template selfadjointView<Eigen::Lower>() * p.B_[k];

		if(settings_.fixedHessianCorrection)
		{
			if (settings_.epsilon > 1e-10)
				Hi_[k] = H_[k] + settings_.epsilon*ControlMatrix::Identity();
			else
				Hi_[k] = H_[k];

			if (settings_.recordSmallestEigenvalue)
			{
				// compute eigenvalues with eigenvectors enabled
				eigenvalueSolver_.compute(Hi_[k], Eigen::ComputeEigenvectors);
				const ControlMatrix& V = eigenvalueSolver_.eigenvectors().real();
				const ControlVector& lambda = eigenvalueSolver_.eigenvalues();

				smallestEigenvalue_ = std::min(smallestEigenvalue_, lambda.minCoeff());

				// Corrected Eigenvalue Matrix
				ControlMatrix D = ControlMatrix::Zero();
				// make D positive semi-definite (as described in IV. B.)
				D.diagonal() = lambda.cwiseMax(settings_.epsilon);

				// reconstruct H
				ControlMatrix Hi_regular = V * D * V.transpose();

				// invert D
				ControlMatrix D_inverse = ControlMatrix::Zero();
				// eigenvalue-wise inversion
				D_inverse.diagonal() = -1.0 * D.diagonal().cwiseInverse();
				ControlMatrix Hi_inverse_regular = V * D_inverse * V.transpose();

				if (!Hi_inverse_[k].isApprox(Hi_inverse_regular, 1e-4))
				{
					std::cout << "warning, inverses not identical at "<<k<<std::endl;
					std::cout << "Hi_inverse_fixed - Hi_inverse_regular: "<<std::endl<<Hi_inverse_[k]-Hi_inverse_regular<<std::endl<<std::endl;
				}
			}

			Hi_inverse_[k] = -Hi_[k].template selfadjointView<Eigen::Lower>().llt().solve(ControlMatrix::Identity());

			// calculate FB gain update
			L_[k].noalias() = Hi_inverse_[k].template selfadjointView<Eigen::Lower>() * G_[k];

			// calculate FF update
			lv_[k].noalias() = Hi_inverse_[k].template selfadjointView<Eigen::Lower>() * gv_[k];

		} else {

			// compute eigenvalues with eigenvectors enabled
			eigenvalueSolver_.compute(H_[k], Eigen::ComputeEigenvectors);
			const ControlMatrix& V = eigenvalueSolver_.eigenvectors().real();
			const ControlVector& lambda = eigenvalueSolver_.eigenvalues();

			if (settings_.recordSmallestEigenvalue)
			{
				smallestEigenvalue_ = std::min(smallestEigenvalue_, lambda.minCoeff());
			}

			// Corrected Eigenvalue Matrix
			ControlMatrix D = ControlMatrix::Zero();
			// make D positive semi-definite (as described in IV. B.)
			D.diagonal() = lambda.cwiseMax(settings_.epsilon);

			// reconstruct H
			Hi_[k].noalias() = V * D * V.transpose();

			// invert D
			ControlMatrix D_inverse = ControlMatrix::Zero();
			// eigenvalue-wise inversion
			D_inverse.diagonal() = -1.0 * D.diagonal().cwiseInverse();
			Hi_inverse_[k].noalias() = V * D_inverse * V.transpose();

			// calculate FB gain update
			L_[k].noalias() = Hi_inverse_[k] * G_[k];

			// calculate FF update
			lv_[k].noalias() = Hi_inverse_[k] * gv_[k];
		}
	}

	void logToMatlab()
	{
	#ifdef MATLAB_FULL_LOG

		matFile_.open("GNRiccatiSolver.mat");

		matFile_.put("sv", sv_.toImplementation());
		matFile_.put("S", S_.toImplementation());
		matFile_.put("L", L_.toImplementation());
		matFile_.put("H", H_.toImplementation());
		matFile_.put("Hi_", Hi_.toImplementation());
		matFile_.put("Hi_inverse", Hi_inverse_.toImplementation());
		matFile_.put("G", G_.toImplementation());
		matFile_.put("gv", gv_.toImplementation());

		matFile_.close();
	#endif
	}

	NLOptConSettings settings_;

	ControlVectorArray gv_;
	FeedbackArray G_;

	ControlMatrixArray H_;
	ControlMatrixArray Hi_;
	ControlMatrixArray Hi_inverse_;
	ControlMatrix H_corrFix_;

	ControlVectorArray lv_;
	FeedbackArray L_;

	StateVectorArray sv_;
	StateMatrixArray S_;

	int N_;

	SCALAR smallestEigenvalue_;

	//! Eigenvalue solver, used for inverting the Hessian and for regularization
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM>> eigenvalueSolver_;

	//! if building with MATLAB support, include matfile
#ifdef MATLAB_FULL_LOG
	matlab::MatFile matFile_;
#endif //MATLAB

};


}
}

#endif /* INCLUDE_CT_OPTCON_LQ_GNRICCATISOLVER_HPP_ */
