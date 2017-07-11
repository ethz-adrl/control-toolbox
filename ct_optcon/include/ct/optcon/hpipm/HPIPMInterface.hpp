/*
 * HPIPMInterface.hpp
 *
 *  Created on: Jul 7, 2017
 *      Author: neunertm
 */

#ifndef INCLUDE_CT_OPTCON_HPIPM_HPIPMINTERFACE_HPP_
#define INCLUDE_CT_OPTCON_HPIPM_HPIPMINTERFACE_HPP_

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

typedef double REAL;

//extern "C"
//{
//int d_memsize_ocp_qp(int N, int *nx, int *nu, int *nb, int *ng);
//void d_create_ocp_qp(int N, int *nx, int *nu, int *nb, int *ng, struct OCP_QP *qp, void *memory);
//
//int d_memsize_ocp_qp_sol(int N, int *nx, int *nu, int *nb, int *ng);
//void d_create_ocp_qp_sol(int N, int *nx, int *nu, int *nb, int *ng, struct OCP_QP_SOL *qp_sol, void *memory);
//
//void d_create_ipm_hard_ocp_qp(struct OCP_QP *qp, struct IPM_HARD_OCP_QP_ARG *arg, struct IPM_HARD_OCP_QP_WORKSPACE *workspace, void *mem);
//void d_solve_ipm2_hard_ocp_qp(struct OCP_QP *qp, struct OCP_QP_SOL *qp_sol, struct IPM_HARD_OCP_QP_WORKSPACE *ws);
//
//void d_cvt_colmaj_to_ocp_qp(REAL **A, REAL **B, REAL **b, REAL **Q, REAL **S, REAL **R, REAL **q, REAL **r, int **idxb, REAL **d_lb, REAL **d_ub, REAL **C, REAL **D, REAL **d_lg, REAL **d_ug, struct OCP_QP *qp);
//void d_cvt_ocp_qp_sol_to_colmaj(struct OCP_QP *qp, struct OCP_QP_SOL *qp_sol, REAL **u, REAL **x, REAL **pi, REAL **lam_lb, REAL **lam_ub, REAL **lam_lg, REAL **lam_ug);
//}


namespace ct {
namespace optcon {

template <int STATE_DIM, int CONTROL_DIM>
class HPIPMInterface
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

	void changeTimeHorizon(int N)
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

	void setProblem(
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
			std::cout << "A["<<i<<"]: "<<std::endl << A[i] << std::endl << std::endl << std::endl;
			std::cout << "B["<<i<<"]: "<<std::endl << B[i] << std::endl << std::endl << std::endl;
			std::cout << "b["<<i<<"]: "<<b[i].transpose() << std::endl << std::endl;
		}

		hb0_ = b[0] + A[0] * x[0];
		hb_[0] = hb0_.data();

		std::cout << "x0 "<<x[0].transpose() << std::endl << std::endl;
		std::cout << "b0: "<<hb0_.transpose() << std::endl << std::endl;

		for (size_t i=0; i<N_; i++)
		{
			hQ_[i] = Q[i].data();
			hS_[i] = P[i].data();
			hR_[i] = R[i].data();
			hqEigen_[i] = qv[i] - Q[i]*x[i] - P[i].transpose()*u[i];
			hq_[i] = hqEigen_[i].data();
			hrEigen_[i] = rv[i] - R[i]*u[i] - P[i]*x[i];
			hr_[i] = hrEigen_[i].data();

			std::cout << "Q["<<i<<"]: "<<Q[i]<<std::endl << std::endl;
			std::cout << "R["<<i<<"]: "<<R[i]<<std::endl << std::endl;

			std::cout << "S["<<i<<"]: "<<P[i]<<std::endl << std::endl;

			std::cout << "q["<<i<<"]: "<<hqEigen_[i].transpose()<<std::endl << std::endl;
			std::cout << "r["<<i<<"]: "<<hrEigen_[i].transpose()<<std::endl << std::endl;
		}
		hQ_[N_] = Q[N_].data();
		hS_[N_] = nullptr;
		hR_[N_] = nullptr;
		hqEigen_[N_] = qv[N_] - Q[N_]*x[N_];
		hq_[N_] = hqEigen_[N_].data();
		hr_[N_] = nullptr;

		hr0_ = hrEigen_[0] + P[0] * x[0];
		hr_[0] = hr0_.data();



		std::cout << "r0: "<<hr0_.transpose() << std::endl;
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


	void solve()
	{
		::d_cvt_colmaj_to_ocp_qp(&hA_[0], &hB_[0], &hb_[0], &hQ_[0], &hS_[0], &hR_[0], &hq_[0], &hr_[0], &hidxb_[0], &hd_lb_[0], &hd_ub_[0], &hC_[0], &hD_[0], &hd_lg_[0], &hd_ug_[0], &qp_);

		::d_solve_ipm2_hard_ocp_qp(&qp_, &qp_sol_, &workspace_);
	}


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



private:
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
};


}
}



#endif /* INCLUDE_CT_OPTCON_HPIPM_HPIPMINTERFACE_HPP_ */
