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

#include "LQOCSolver.hpp"

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

	HPIPMInterface(int N = -1);

	~HPIPMInterface();

	virtual void configure(const NLOptConSettings& settings) override;

	void solve() override;

	virtual void computeStateAndControlUpdates() override;

	virtual ct::core::StateVectorArray<STATE_DIM> getSolutionState() override;

	virtual ct::core::ControlVectorArray<CONTROL_DIM> getSolutionControl() override;

	virtual void getFeedback(ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM>& K) override;

	virtual ct::core::ControlVectorArray<CONTROL_DIM> getFeedforwardUpdates() override;

	void printSolution();


private:

	void setProblemImpl(std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem) override;

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
			ControlMatrixArray& R,
			bool keepPointers = false);

	void changeNumberOfStages(int N);

	void d_zeros(double **pA, int row, int col);

	void d_print_mat(int m, int n, double *A, int lda);

	/* prints a matrix in column-major format (exponential notation) */
	void d_print_e_mat(int m, int n, double *A, int lda);

	void d_print_e_tran_mat(int row, int col, double *A, int lda);

	int N_; // horizon length

	std::vector<int> nx_; // number of states per stage
	std::vector<int> nu_; // number of inputs per stage

	std::vector<int> nb_; // number of box constraints per stage, currently always zero
	std::vector<int> ng_; // number of general constraints per stage, currently always zero


	std::vector<double*> hA_; // system state sensitivities
	std::vector<double*> hB_; // system input sensitivities
	StateVectorArray bEigen_; // for transcription
	std::vector<double*> hb_;
	Eigen::Matrix<double, state_dim, 1> hb0_;

	std::vector<double*> hQ_; // pure state penalty hessian
	std::vector<double*> hS_; // cross-terms
	std::vector<double*> hR_; // pure control penalty hessian
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

	// solution
	std::vector<double*> u_;
	std::vector<double*> x_;
	std::vector<double*> pi_;
	std::vector<double*> lam_lb_;
	std::vector<double*> lam_ub_;
	std::vector<double*> lam_lg_;
	std::vector<double*> lam_ug_;

	ct::core::StateVectorArray<STATE_DIM> hx_;
	ct::core::StateVectorArray<STATE_DIM> hpi_;
	ct::core::ControlVectorArray<CONTROL_DIM> hu_;

	std::vector<char> qp_mem_;
	struct d_ocp_qp qp_;

	std::vector<char> qp_sol_mem_;
	struct d_ocp_qp_sol qp_sol_;

	struct d_ipm_hard_ocp_qp_arg arg_;
	std::vector<char> ipm_mem_;
	struct d_ipm_hard_ocp_qp_workspace workspace_;

	NLOptConSettings settings_;
};


} // optcon
} // ct

#endif

#endif /* INCLUDE_CT_OPTCON_HPIPM_HPIPMINTERFACE_HPP_ */
