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

#ifndef INCLUDE_CT_OPTCON_HPIPM_HPIPMINTERFACE_HPP_
#define INCLUDE_CT_OPTCON_HPIPM_HPIPMINTERFACE_HPP_

#ifdef HPIPM

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_v_aux_ext_dep.h>
#include <blasfeo_d_aux_ext_dep.h>
#include <blasfeo_i_aux_ext_dep.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_blas.h>

extern "C"
{
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_sol.h>
#include <hpipm_d_ocp_qp_ipm_hard.h>
}

#include <unsupported/Eigen/MatrixFunctions>


namespace ct {
namespace optcon {

template <int STATE_DIM, int CONTROL_DIM>
class HPIPMInterface : public LQOCSolver<STATE_DIM, CONTROL_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const int state_dim = STATE_DIM;
	static const int control_dim = CONTROL_DIM;

	typedef ct::core::StateMatrix<STATE_DIM> StateMatrix;
	typedef ct::core::StateMatrixArray<STATE_DIM> StateMatrixArray;
	typedef ct::core::ControlMatrix<CONTROL_DIM> ControlMatrix;
	typedef ct::core::ControlMatrixArray<CONTROL_DIM> ControlMatrixArray;
	typedef ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM> StateControlMatrixArray;
	typedef ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM> FeedbackArray;

	typedef ct::core::StateVectorArray<STATE_DIM> StateVectorArray;
	typedef ct::core::ControlVectorArray<CONTROL_DIM> ControlVectorArray;

	HPIPMInterface() :
		x0_(nullptr),
		N_(-1)
	{
		// some zero variables
		hb0_.setZero();
		hr0_.setZero();

		arg_.alpha_min = 1e-8;
		arg_.mu_max = 1e-12;
		arg_.iter_max = 20;
		arg_.mu0 = 2.0;

		//changeTimeHorizon(N);
	}

	~HPIPMInterface()
	{
	}


	virtual void configure(const NLOptConSettings& settings) override
	{
		settings_ = settings;
	}

	void solve() override
	{
//		for (size_t i=0; i<N_+1; i++)
//		{
//			if (i<N_)
//			{
//				printf("\nA\n");
//				d_print_mat(STATE_DIM, STATE_DIM, hA_[i], STATE_DIM);
//				printf("\nB\n");
//				d_print_mat(STATE_DIM, CONTROL_DIM, hB_[i], STATE_DIM);
//				printf("\nb\n");
//				d_print_mat(1, STATE_DIM, hb_[i], 1);
//			}
//
//			printf("\nQ\n");
//			d_print_mat(STATE_DIM, STATE_DIM, hQ_[i], STATE_DIM);
//			printf("\nq\n");
//			d_print_mat(1, STATE_DIM, hq_[i], 1);
//
//
//			if (i<N_)
//			{
//				printf("\nR\n");
//				d_print_mat(CONTROL_DIM, CONTROL_DIM, hR_[i], CONTROL_DIM);
//				printf("\nS\n");
//				d_print_mat(CONTROL_DIM, STATE_DIM, hS_[i], CONTROL_DIM);
//				printf("\nr\n");
//				d_print_mat(1, CONTROL_DIM, hr_[i], 1);
//			}
//		}

		::d_cvt_colmaj_to_ocp_qp(hA_.data(), hB_.data(), hb_.data(), hQ_.data(), hS_.data(), hR_.data(), hq_.data(), hr_.data(), hidxb_.data(), hd_lb_.data(), hd_ub_.data(), hC_.data(), hD_.data(), hd_lg_.data(), hd_ug_.data(), &qp_);
		::d_solve_ipm2_hard_ocp_qp(&qp_, &qp_sol_, &workspace_);
	}

	virtual ct::core::StateVectorArray<STATE_DIM> getSolutionState() override { throw std::runtime_error("not implemented"); }
	virtual ct::core::ControlVectorArray<CONTROL_DIM> getSolutionControl() override { throw std::runtime_error("not implemented"); }
	virtual ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM> getFeedback() override { throw std::runtime_error("not implemented"); }


	void printSolution()
	{
		int ii;

		double *u[N_+1]; for(ii=0; ii<=N_; ii++) d_zeros(u+ii, nu_[ii], 1);
		double *x[N_+1]; for(ii=0; ii<=N_; ii++) d_zeros(x+ii, nx_[ii], 1);
		double *pi[N_]; for(ii=0; ii<N_; ii++) d_zeros(pi+ii, nx_[ii+1], 1);
		double *lam_lb[N_+1]; for(ii=0; ii<=N_; ii++) d_zeros(lam_lb+ii, nb_[ii], 1);
		double *lam_ub[N_+1]; for(ii=0; ii<=N_; ii++) d_zeros(lam_ub+ii, nb_[ii], 1);
		double *lam_lg[N_+1]; for(ii=0; ii<=N_; ii++) d_zeros(lam_lg+ii, ng_[ii], 1);
		double *lam_ug[N_+1]; for(ii=0; ii<=N_; ii++) d_zeros(lam_ug+ii, ng_[ii], 1);

		::d_cvt_ocp_qp_sol_to_colmaj(&qp_, &qp_sol_, u, x, pi, lam_lb, lam_ub, lam_lg, lam_ug);


			printf("\nsolution\n\n");
			printf("\nu\n");
			for(ii=0; ii<=N_; ii++)
				d_print_mat(1, nu_[ii], u[ii], 1);
			printf("\nx\n");
			for(ii=0; ii<=N_; ii++)
				d_print_mat(1, nx_[ii], x[ii], 1);

			/*
		#if 1
			printf("\npi\n");
			for(ii=0; ii<N_; ii++)
				d_print_mat(1, nx_[ii+1], pi[ii], 1);
			printf("\nlam_lb\n");
			for(ii=0; ii<=N_; ii++)
				d_print_mat(1, nb_[ii], lam_lb[ii], 1);
			printf("\nlam_ub\n");
			for(ii=0; ii<=N_; ii++)
				d_print_mat(1, nb_[ii], lam_ub[ii], 1);
			printf("\nlam_lg\n");
			for(ii=0; ii<=N_; ii++)
				d_print_mat(1, ng_[ii], lam_lg[ii], 1);
			printf("\nlam_ug\n");
			for(ii=0; ii<=N_; ii++)
				d_print_mat(1, ng_[ii], lam_ug[ii], 1);

			printf("\nt_lb\n");
			for(ii=0; ii<=N_; ii++)
				d_print_mat(1, nb_[ii], (qp_sol_.t_lb+ii)->pa, 1);
			printf("\nt_ub\n");
			for(ii=0; ii<=N_; ii++)
				d_print_mat(1, nb_[ii], (qp_sol_.t_ub+ii)->pa, 1);
			printf("\nt_lg\n");
			for(ii=0; ii<=N_; ii++)
				d_print_mat(1, ng_[ii], (qp_sol_.t_lg+ii)->pa, 1);
			printf("\nt_ug\n");
			for(ii=0; ii<=N_; ii++)
				d_print_mat(1, ng_[ii], (qp_sol_.t_ug+ii)->pa, 1);

			printf("\nresiduals\n\n");
			printf("\nres_g\n");
			for(ii=0; ii<=N_; ii++)
				d_print_e_mat(1, nu_[ii]+nx_[ii], (workspace_.res_g+ii)->pa, 1);
			printf("\nres_b\n");
			for(ii=0; ii<N_; ii++)
				d_print_e_mat(1, nx_[ii+1], (workspace_.res_b+ii)->pa, 1);
			printf("\nres_m_lb\n");
			for(ii=0; ii<=N_; ii++)
				d_print_e_mat(1, nb_[ii], (workspace_.res_m_lb+ii)->pa, 1);
			printf("\nres_m_ub\n");
			for(ii=0; ii<=N_; ii++)
				d_print_e_mat(1, nb_[ii], (workspace_.res_m_ub+ii)->pa, 1);
			printf("\nres_m_lg\n");
			for(ii=0; ii<=N_; ii++)
				d_print_e_mat(1, ng_[ii], (workspace_.res_m_lg+ii)->pa, 1);
			printf("\nres_m_ug\n");
			for(ii=0; ii<=N_; ii++)
				d_print_e_mat(1, ng_[ii], (workspace_.res_m_ug+ii)->pa, 1);
			printf("\nres_d_lb\n");
			for(ii=0; ii<=N_; ii++)
				d_print_e_mat(1, nb_[ii], (workspace_.res_d_lb+ii)->pa, 1);
			printf("\nres_d_ub\n");
			for(ii=0; ii<=N_; ii++)
				d_print_e_mat(1, nb_[ii], (workspace_.res_d_ub+ii)->pa, 1);
			printf("\nres_d_lg\n");
			for(ii=0; ii<=N_; ii++)
				d_print_e_mat(1, ng_[ii], (workspace_.res_d_lg+ii)->pa, 1);
			printf("\nres_d_ug\n");
			for(ii=0; ii<=N_; ii++)
				d_print_e_mat(1, ng_[ii], (workspace_.res_d_ug+ii)->pa, 1);
			printf("\nres_mu\n");
			printf("\n%e\n\n", workspace_.res_mu);
		#endif
		*/

			printf("\nipm iter = %d\n", workspace_.iter);
			printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha\t\tmu\n");
			::d_print_e_tran_mat(5, workspace_.iter, workspace_.stat, 5);
	}

	void solveLinearProblem(
			ct::core::StateVector<STATE_DIM>& x0,
			ct::core::LinearSystem<STATE_DIM, CONTROL_DIM>& linearSystem,
			ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>& costFunction,
			ct::core::StateVector<STATE_DIM>& stateOffset,
			double dt
	)
	{
		ct::core::ControlVector<CONTROL_DIM> uNom; uNom.setZero(); // reference control should not matter for linear system

		StateMatrix Ac = linearSystem.getDerivativeState(x0, uNom, 0);
		StateMatrix Adt = dt * Ac;

		StateMatrixArray A(N_, Adt.exp());
		StateControlMatrixArray B(N_, Ac.inverse() * (A[0] - StateMatrix::Identity()) *  linearSystem.getDerivativeControl(x0, uNom, 0));

//		Eigen::Matrix<double, STATE_DIM, 1> bd;
//		bd.setConstant(0.1);
//
//		Eigen::Matrix<double, STATE_DIM, 1> bc;
//		bc = (A[0] - StateMatrix::Identity()).inverse() * Ac * bd;
//
//		std::cout << "bc: " << bc.transpose() << std::endl;
//
//
//
//		Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> Brec;
//		Brec = (A[0] - StateMatrix::Identity()).inverse() * Ac * B[0];
//
//		std::cout << "Brec: " << std::endl << Brec << std::endl;
//		std::cout << "Bc: " << std::endl << linearSystem.getDerivativeControl(x0, uNom, 0) << std::endl;



		// feed current state and control to cost function
		costFunction.setCurrentStateAndControl(x0, uNom, 0);

		// derivative of cost with respect to state
		StateVectorArray qv(N_+1, costFunction.stateDerivativeIntermediate()*dt);
		StateMatrixArray Q(N_+1, costFunction.stateSecondDerivativeIntermediate()*dt);

		// derivative of cost with respect to control and state
		FeedbackArray P(N_, costFunction.stateControlDerivativeIntermediate()*dt);

		// derivative of cost with respect to control
		ControlVectorArray rv(N_, costFunction.controlDerivativeIntermediate()*dt);

		ControlMatrixArray R(N_, costFunction.controlSecondDerivativeIntermediate()*dt);

		StateVectorArray b(N_+1, stateOffset);

		StateVectorArray x(N_+1, x0);
		ControlVectorArray u(N_, uNom);

		setProblem(x, u, A, B, b, P, qv, Q, rv, R);
		solve();

	}



private:
	void setProblemImpl(std::shared_ptr<LQOCProblem>& lqocProblem) override
	{
		changeNumberOfStages(lqocProblem->getNumberOfStages);
		setupHPIPM(
				lqocProblem->x_,
				lqocProblem->u_,
				lqocProblem->A_,
				lqocProblem->B_,
				lqocProblem->b_,
				lqocProblem->P_,
				lqocProblem->qv_,
				lqocProblem->Q_,
				lqocProblem->rv_,
				lqocProblem->R_);
	}


	void setupHPIPM(
			StateVectorArray& x,
			ControlVectorArray& u,
			StateMatrixArray& A,
			StateControlMatrixArray& B,
			StateVectorArray& b,
			FeedbackArray& P,
			StateVectorArray& qv,
			StateMatrixArray& Q,
			ControlVectorArray& rv,
			ControlMatrixArray& R
	)
	{
		if (N_ == -1)
			throw std::runtime_error ("Time horizon not set, please set it first");

		x0_ = x[0].data();

		for (size_t i=0; i<N_; i++)
		{
			hA_[i] = A[i].data();
			hB_[i] = B[i].data();
			hb_[i] = b[i].data();
		}

		hb0_ = b[0] + A[0] * x[0];
		hb_[0] = hb0_.data();

		for (size_t i=0; i<N_; i++)
		{
			hQ_[i] = Q[i].data();
			hS_[i] = P[i].data();
			hR_[i] = R[i].data();
			hqEigen_[i] = qv[i] - Q[i]*x[i] - P[i].transpose()*u[i];
			hq_[i] = hqEigen_[i].data();
			hrEigen_[i] = rv[i] - R[i]*u[i] - P[i]*x[i];
			hr_[i] = hrEigen_[i].data();
		}
		hQ_[N_] = Q[N_].data();
		hS_[N_] = nullptr;
		hR_[N_] = nullptr;
		hqEigen_[N_] = qv[N_] - Q[N_]*x[N_];
		hq_[N_] = hqEigen_[N_].data();
		hr_[N_] = nullptr;

		hr0_ = hrEigen_[0] + P[0] * x[0];
		hr_[0] = hr0_.data();

//		printf("\nQ\n");
//		d_print_mat(STATE_DIM, STATE_DIM, hQ_[0], STATE_DIM);
//		printf("\nR\n");
//		d_print_mat(CONTROL_DIM, CONTROL_DIM, hR_[0], CONTROL_DIM);
//		printf("\nS\n");
//		d_print_mat(CONTROL_DIM, STATE_DIM, hS_[0], CONTROL_DIM);
//		printf("\nq\n");
//		d_print_mat(1, STATE_DIM, hq_[0], 1);
//		printf("\nr\n");
//		d_print_mat(1, CONTROL_DIM, hr_[1], 1);
//		printf("\nr0\n");
//		d_print_mat(1, CONTROL_DIM, hr_[0], 1);
	}


	void changeNumberOfStages(int N)
	{
		N_ = N;

		nx_.resize(N_+1, STATE_DIM);
		nu_.resize(N_+1, CONTROL_DIM);
		nb_.resize(N_+1, 0);
		ng_.resize(N_+1, 0);

		hA_.resize(N_);
		hB_.resize(N_);
		hb_.resize(N_);
		hQ_.resize(N_+1);
		hS_.resize(N_+1);
		hR_.resize(N_+1);
		hqEigen_.resize(N_+1);
		hq_.resize(N_+1);
		hrEigen_.resize(N_+1);
		hr_.resize(N_+1);
		hd_lb_.resize(N_+1);
		hd_ub_.resize(N_+1);
		hd_lg_.resize(N_+1);
		hd_ug_.resize(N_+1);
		hC_.resize(N_+1);
		hD_.resize(N_+1);
		hidxb_.resize(N_+1);

		// initial state is not a decision variable but given
		nx_[0] = 0;

		// last input is not a decision variable
		nu_[N] = 0;


		int qp_size = ::d_memsize_ocp_qp(N_, nx_.data(), nu_.data(), nb_.data(), ng_.data());
		std::cout << "qp_size: " << qp_size << std::endl;
		qp_mem_.resize(qp_size);
		::d_create_ocp_qp(N_, nx_.data(), nu_.data(), nb_.data(), ng_.data(), &qp_, qp_mem_.data());

		int qp_sol_size = ::d_memsize_ocp_qp_sol(N_, nx_.data(), nu_.data(), nb_.data(), ng_.data());
		std::cout << "qp_sol_size: " << qp_sol_size << std::endl;
		qp_sol_mem_.resize(qp_sol_size);
		::d_create_ocp_qp_sol(N_, nx_.data(), nu_.data(), nb_.data(), ng_.data(), &qp_sol_, qp_sol_mem_.data());

		int ipm_size = ::d_memsize_ipm_hard_ocp_qp(&qp_, &arg_);
		std::cout << "ipm_size: " << ipm_size << std::endl;
		ipm_mem_.resize(ipm_size);
		::d_create_ipm_hard_ocp_qp(&qp_, &arg_, &workspace_, ipm_mem_.data());
	}



	void d_zeros(double **pA, int row, int col)
	{
		*pA = (double*)malloc((row*col)*sizeof(double));
		double *A = *pA;
		int i;
		for(i=0; i<row*col; i++) A[i] = 0.0;
	}
	void d_print_mat(int m, int n, double *A, int lda)
	{
	int i, j;
	for(i=0; i<m; i++)
		{
		for(j=0; j<n; j++)
			{
			printf("%9.5f ", A[i+lda*j]);
			}
		printf("\n");
		}
	printf("\n");
	}
	/* prints the transposed of a matrix in column-major format */
	void d_print_tran_mat(int row, int col, double *A, int lda)
	{
	int i, j;
	for(j=0; j<col; j++)
		{
		for(i=0; i<row; i++)
			{
			printf("%9.5f ", A[i+lda*j]);
			}
		printf("\n");
		}
	printf("\n");
	}

	/* prints a matrix in column-major format (exponential notation) */
	void d_print_e_mat(int m, int n, double *A, int lda)
	{
	int i, j;
	for(i=0; i<m; i++)
		{
		for(j=0; j<n; j++)
			{
			printf("%e\t", A[i+lda*j]);
			}
		printf("\n");
		}
	printf("\n");
	}

	void d_print_e_tran_mat(int row, int col, double *A, int lda)
	{
	int i, j;
	for(j=0; j<col; j++)
		{
		for(i=0; i<row; i++)
			{
			printf("%e\t", A[i+lda*j]);
			}
		printf("\n");
		}
	printf("\n");
	}


	int N_; // horizon length

	std::vector<int> nx_; // number of states per stage
	std::vector<int> nu_; // number of inputs per stage

	std::vector<int> nb_; // number of bounds, currently always zero
	std::vector<int> ng_; // number of general constraints, currently always zero


	std::vector<double*> hA_; // system state sensitivities
	std::vector<double*> hB_; // system input sensitivities
	std::vector<double*> hb_;
	Eigen::Matrix<double, state_dim, 1> hb0_;

	std::vector<double*> hQ_;
	std::vector<double*> hS_;
	std::vector<double*> hR_;
	StateVectorArray hqEigen_;
	std::vector<double*> hq_;
	ControlVectorArray hrEigen_;
	std::vector<double*> hr_;
	Eigen::Matrix<double, control_dim, 1> hr0_;

	std::vector<double*> hd_lb_;
	std::vector<double*> hd_ub_;
	std::vector<double*> hd_lg_;
	std::vector<double*> hd_ug_;
	std::vector<double*> hC_;
	std::vector<double*> hD_;
	std::vector<int*> hidxb_;
	double *x0_; // initial state


	std::vector<char> qp_mem_;
	struct d_ocp_qp qp_;

	std::vector<char> qp_sol_mem_;
	struct d_ocp_qp_sol qp_sol_;

	struct d_ipm_hard_ocp_qp_arg arg_;
	std::vector<char> ipm_mem_;
	struct d_ipm_hard_ocp_qp_workspace workspace_;

	NLOptConSettings_t settings_;
};


}
}

#endif

#endif /* INCLUDE_CT_OPTCON_HPIPM_HPIPMINTERFACE_HPP_ */
