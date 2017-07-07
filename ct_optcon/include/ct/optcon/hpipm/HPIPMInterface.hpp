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

	HPIPMInterface(int N):
		N_(N),

		nx_(N_+1, STATE_DIM),
		nu_(N_+1, CONTROL_DIM),
		nb_(N_+1, 0),
		ng_(N_+1, 0),

		hA_(N),
		hB_(N),
		hb_(N),
		hQ_(N_+1),
		hS_(N_+1),
		hR_(N_+1),
		hq_(N_+1),
		hr_(N_+1),
		hd_lb_(N_+1),
		hd_ub_(N_+1),
		hd_lg_(N_+1),
		hd_ug_(N_+1),
		hC_(N_+1),
		hD_(N_+1),
		hidxb_(N_+1),
		x0_(nullptr)
	{
		// initial state is not a decision variable but given
		nx_[0] = 0;

		// last input is not a decision variable
		nu_[N] = 0;

		// some zero variables
		hb0_.setZero();
		hr0_.setZero();

		int qp_size = ::d_memsize_ocp_qp(N_, &nx_[0], &nu_[0], &nb_[0], &ng_[0]);
		qp_mem_ = malloc(qp_size);
		::d_create_ocp_qp(N_, &nx_[0], &nu_[0], &nb_[0], &ng_[0], &qp_, qp_mem_);

		int qp_sol_size = ::d_memsize_ocp_qp_sol(N_, &nx_[0], &nu_[0], &nb_[0], &ng_[0]);
		qp_sol_mem_ = malloc(qp_sol_size);
		::d_create_ocp_qp_sol(N_, &nx_[0], &nu_[0], &nb_[0], &ng_[0], &qp_sol_, qp_sol_mem_);

		arg_.alpha_min = 1e-8;
		arg_.mu_max = 1e-12;
		arg_.iter_max = 20;
		arg_.mu0 = 2.0;
		int ipm_size = ::d_memsize_ipm_hard_ocp_qp(&qp_, &arg_);
		ipm_mem_ = malloc(ipm_size);

		::d_create_ipm_hard_ocp_qp(&qp_, &arg_, &workspace_, ipm_mem_);
	}

	void setProblem(
			ct::core::StateVector<STATE_DIM>& x0,
			StateMatrixArray& A,
			StateControlMatrixArray& B,
			FeedbackArray& P,
			StateVectorArray& qv,
			StateMatrixArray& Q,
			ControlVectorArray& rv,
			ControlMatrixArray& R
	)
	{
		std::cout << "A: " << std::endl << A[0] << std::endl;
		std::cout << "B: " << std::endl << B[0] << std::endl;


		std::cout << "Q: "<<std::endl<<Q[0]<<std::endl;
		std::cout << "R: "<<std::endl<<R[0]<<std::endl;
		std::cout << "qv: "<<qv[0].transpose()<<std::endl;
		std::cout << "rv: "<<rv[0].transpose()<<std::endl;

		x0_ = x0.data();

		for (size_t i=0; i<N_; i++)
		{
			hA_[i] = A[i].data();
			hB_[i] = B[i].data();
			hb_[i] = hb0_.data();
		}
		hb0_ = A[0] * x0;
		hb_[0] = hb0_.data();

		for (size_t i=0; i<N_+1; i++)
		{
			hQ_[i] = Q[i].data();
			hS_[i] = P[i].data();
			hR_[i] = R[i].data();
			hq_[i] = qv[i].data();
			hr_[i] = rv[i].data();
		}
		hr0_ = P[0] * x0;
		hr_[0] = hr0_.data();
	}

	void setLinearProblem(
			ct::core::StateVector<STATE_DIM>& x0,
			ct::core::LinearSystem<STATE_DIM, CONTROL_DIM>& linearSystem,
			ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>& costFunction,
			double dt
	)
	{
		ct::core::StateVector<STATE_DIM> xNom; xNom.setZero(); // reference state should not matter for linear system
		ct::core::ControlVector<CONTROL_DIM> uNom; uNom.setZero(); // reference control should not matter for linear system

		/*
		StateMatrixArray A(N_, StateMatrix::Identity() + dt * linearSystem.getDerivativeState(x0, uNom, 0));
		StateControlMatrixArray B(N_, dt * linearSystem.getDerivativeControl(x0, uNom, 0));
		*/

		StateMatrix Ac = linearSystem.getDerivativeState(x0, uNom, 0);
		StateMatrix Adt = dt * Ac;

		StateMatrixArray A(N_, Adt.exp());
		StateControlMatrixArray B(N_, Ac.inverse() * (A[0] - StateMatrix::Identity()) *  linearSystem.getDerivativeControl(x0, uNom, 0));


		// feed current state and control to cost function
		costFunction.setCurrentStateAndControl(xNom, uNom, 0);

		// derivative of cost with respect to state
		StateVectorArray qv(N_+1, costFunction.stateDerivativeIntermediate()*dt);
		StateMatrixArray Q(N_+1, costFunction.stateSecondDerivativeIntermediate()*dt);

		// derivative of cost with respect to control and state
		FeedbackArray P(N_+1, costFunction.stateControlDerivativeIntermediate()*dt);

		// derivative of cost with respect to control
		ControlVectorArray rv(N_+1, costFunction.controlDerivativeIntermediate()*dt);

		ControlMatrixArray R(N_+1, costFunction.controlSecondDerivativeIntermediate()*dt);

		setProblem(x0, A, B, P, qv, Q, rv, R);
	}


	void solve()
	{
		int ii;

		::d_cvt_colmaj_to_ocp_qp(&hA_[0], &hB_[0], &hb_[0], &hQ_[0], &hS_[0], &hR_[0], &hq_[0], &hr_[0], &hidxb_[0], &hd_lb_[0], &hd_ub_[0], &hC_[0], &hD_[0], &hd_lg_[0], &hd_ug_[0], &qp_);

		::d_solve_ipm2_hard_ocp_qp(&qp_, &qp_sol_, &workspace_);


		double *u[N_+1]; for(ii=0; ii<=N_; ii++) d_zeros(u+ii, nu_[ii], 1);
		double *x[N_+1]; for(ii=0; ii<=N_; ii++) d_zeros(x+ii, nx_[ii], 1);
		double *pi[N_]; for(ii=0; ii<N_; ii++) d_zeros(pi+ii, nx_[ii+1], 1);
		double *lam_lb[N_+1]; for(ii=0; ii<=N_; ii++) d_zeros(lam_lb+ii, nb_[ii], 1);
		double *lam_ub[N_+1]; for(ii=0; ii<=N_; ii++) d_zeros(lam_ub+ii, nb_[ii], 1);
		double *lam_lg[N_+1]; for(ii=0; ii<=N_; ii++) d_zeros(lam_lg+ii, ng_[ii], 1);
		double *lam_ug[N_+1]; for(ii=0; ii<=N_; ii++) d_zeros(lam_ug+ii, ng_[ii], 1);

		::d_cvt_ocp_qp_sol_to_colmaj(&qp_, &qp_sol_, u, x, pi, lam_lb, lam_ub, lam_lg, lam_ug);

		#if 1
			printf("\nsolution\n\n");
			printf("\nu\n");
			for(ii=0; ii<=N_; ii++)
				d_print_mat(1, nu_[ii], u[ii], 1);
			printf("\nx\n");
			for(ii=0; ii<=N_; ii++)
				d_print_mat(1, nx_[ii], x[ii], 1);
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
	std::vector<double*> hq_;
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


	void *qp_mem_;
	struct d_ocp_qp qp_;

	void *qp_sol_mem_;
	struct d_ocp_qp_sol qp_sol_;

	struct d_ipm_hard_ocp_qp_arg arg_;
	void *ipm_mem_;
	struct d_ipm_hard_ocp_qp_workspace workspace_;
};


}
}



#endif /* INCLUDE_CT_OPTCON_HPIPM_HPIPMINTERFACE_HPP_ */
