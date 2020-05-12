/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {


template <typename MANIFOLD, size_t CONTROL_DIM>
GNRiccatiSolver<MANIFOLD, CONTROL_DIM>::GNRiccatiSolver(const std::shared_ptr<LQOCProblem_t>& lqocProblem)
    : LQOCSolver<MANIFOLD, CONTROL_DIM>(lqocProblem), N_(-1)
{
    Eigen::initParallel();
    Eigen::setNbThreads(settings_.nThreadsEigen);
}


template <typename MANIFOLD, size_t CONTROL_DIM>
GNRiccatiSolver<MANIFOLD, CONTROL_DIM>::GNRiccatiSolver(int N)
{
    changeNumberOfStages(N);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void GNRiccatiSolver<MANIFOLD, CONTROL_DIM>::solve()
{
    for (int i = this->lqocProblem_->getNumberOfStages() - 1; i >= 0; i--)
        solveSingleStage(i);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void GNRiccatiSolver<MANIFOLD, CONTROL_DIM>::solveSingleStage(int N)
{
    if (N == this->lqocProblem_->getNumberOfStages() - 1)
        initializeCostToGo();

    designController(N);

    if (N > 0)
        computeCostToGo(N);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void GNRiccatiSolver<MANIFOLD, CONTROL_DIM>::configure(const NLOptConSettings& settings)
{
    settings_ = settings;
    H_corrFix_ = settings_.epsilon * ControlMatrix::Identity();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void GNRiccatiSolver<MANIFOLD, CONTROL_DIM>::computeStatesAndControls()
{
    LQOCProblem_t& p = *this->lqocProblem_;

    this->x_sol_[0].setZero();  // should always be zero (fixed init state)

    for (int k = 0; k < this->lqocProblem_->getNumberOfStages(); k++)
    {
        //! control update rule in diff coordinates
        this->u_sol_[k] = this->lv_[k] + this->L_[k] * this->x_sol_[k];

        //! state update rule in diff coordinates
        StateMatrix A_orig = p.Adj_x_[k + 1] * p.A_[k];                 // A "without trick for backward pass" //TODO: document this
        StateControlMatrix B_orig = p.Adj_x_[k + 1] * p.B_[k];          // B "without trick for backward pass"
        typename MANIFOLD::Tangent b_orig = p.Adj_x_[k + 1] * p.b_[k];  // b "without trick for backward pass"
        // Note that we need to transport the state update into the tagent space of k+1
        this->x_sol_[k + 1] =
            p.Adj_x_[k + 1].transpose() * (A_orig * this->x_sol_[k] + B_orig * (this->u_sol_[k]) + b_orig);
    }
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void GNRiccatiSolver<MANIFOLD, CONTROL_DIM>::computeFeedbackMatrices()
{ /*no action required, already computed in backward pass */
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void GNRiccatiSolver<MANIFOLD, CONTROL_DIM>::compute_lv()
{ /*no action required, already computed in backward pass*/
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto GNRiccatiSolver<MANIFOLD, CONTROL_DIM>::getSmallestEigenvalue() -> SCALAR
{
    return smallestEigenvalue_;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void GNRiccatiSolver<MANIFOLD, CONTROL_DIM>::setProblemImpl(std::shared_ptr<LQOCProblem_t> lqocProblem)
{
    if (lqocProblem->isConstrained())
    {
        throw std::runtime_error(
            "Selected wrong solver - GNRiccatiSolver cannot handle constrained problems. Use a different solver");
    }

    const int& N = lqocProblem->getNumberOfStages();
    changeNumberOfStages(N);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void GNRiccatiSolver<MANIFOLD, CONTROL_DIM>::changeNumberOfStages(int N)
{
    if (N <= 0)
        return;

    if (N_ == N)
        return;

    gv_.resize(N);
    G_.resize(N);

    H_.resize(N);
    Hi_.resize(N);
    Hi_inverse_.resize(N);

    this->lv_.resize(N);
    this->L_.resize(N);

    this->x_sol_.resize(N + 1);
    this->u_sol_.resize(N);

    sv_.resize(N + 1);
    S_.resize(N + 1);

    N_ = N;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void GNRiccatiSolver<MANIFOLD, CONTROL_DIM>::initializeCostToGo()
{
    //! since intializeCostToGo is the first call, we initialize the smallestEigenvalue here.
    smallestEigenvalue_ = std::numeric_limits<SCALAR>::infinity();

    // initialize quadratic approximation of cost to go
    const int& N = this->lqocProblem_->getNumberOfStages();
    LQOCProblem_t& p = *this->lqocProblem_;

    S_[N] = p.Q_[N];
    sv_[N] = p.qv_[N];
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void GNRiccatiSolver<MANIFOLD, CONTROL_DIM>::computeCostToGo(size_t k)
{
    LQOCProblem_t& p = *this->lqocProblem_;

    S_[k] = p.Q_[k];
    S_[k].noalias() += p.A_[k].transpose() * S_[k + 1] * p.A_[k];
    S_[k].noalias() -= this->L_[k].transpose() * Hi_[k] * this->L_[k];

    S_[k] = 0.5 * (S_[k] + S_[k].transpose()).eval();

    sv_[k] = p.qv_[k];
    sv_[k]/*.noalias()*/ += p.A_[k].transpose() * sv_[k + 1];
    sv_[k]/*.noalias()*/ += p.A_[k].transpose() * S_[k + 1] * p.b_[k];
    sv_[k]/*.noalias()*/ += this->L_[k].transpose() * Hi_[k] * this->lv_[k];
    sv_[k]/*.noalias()*/ += this->L_[k].transpose() * gv_[k];
    sv_[k]/*.noalias()*/ += G_[k].transpose() * this->lv_[k]; // TODO: bring back all the noalias()
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void GNRiccatiSolver<MANIFOLD, CONTROL_DIM>::designController(size_t k)
{
    LQOCProblem_t& p = *this->lqocProblem_;

    gv_[k] = p.rv_[k];
    gv_[k].noalias() += p.B_[k].transpose() * sv_[k + 1];
    gv_[k].noalias() += p.B_[k].transpose() * S_[k + 1].template selfadjointView<Eigen::Lower>() * p.b_[k];

    G_[k] = p.P_[k];
    //G_[k].noalias() += B_[k].transpose() * S_[k+1] * A_[k];
    G_[k].noalias() += p.B_[k].transpose() * S_[k + 1].template selfadjointView<Eigen::Lower>() * p.A_[k];

    H_[k] = p.R_[k];
    //H_[k].noalias() += B_[k].transpose() * S_[k+1] * B_[k];
    H_[k].noalias() += p.B_[k].transpose() * S_[k + 1].template selfadjointView<Eigen::Lower>() * p.B_[k];

    if (settings_.fixedHessianCorrection)
    {
        if (settings_.epsilon > 1e-10)
            Hi_[k] = H_[k] + settings_.epsilon * ControlMatrix::Identity();
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
                std::cout << "warning, inverses not identical at " << k << std::endl;
                std::cout << "Hi_inverse_fixed - Hi_inverse_regular: " << std::endl
                          << Hi_inverse_[k] - Hi_inverse_regular << std::endl
                          << std::endl;
            }
        }

        Hi_inverse_[k] = -Hi_[k].template selfadjointView<Eigen::Lower>().llt().solve(ControlMatrix::Identity());

        // calculate FB gain update
        this->L_[k].noalias() = Hi_inverse_[k].template selfadjointView<Eigen::Lower>() * G_[k];

        // calculate FF update
        this->lv_[k].noalias() = Hi_inverse_[k].template selfadjointView<Eigen::Lower>() * gv_[k];
    }
    else
    {
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
        this->L_[k].noalias() = Hi_inverse_[k] * G_[k];

        // calculate FF update
        this->lv_[k].noalias() = Hi_inverse_[k] * gv_[k];
    }
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void GNRiccatiSolver<MANIFOLD, CONTROL_DIM>::logToMatlab()
{
#ifdef MATLAB_FULL_LOG

    matFile_.open("GNRiccatiSolver.mat");

    matFile_.put("sv", sv_.toImplementation());
    matFile_.put("S", S_.toImplementation());
    matFile_.put("L", this->L_.toImplementation());
    matFile_.put("H", H_.toImplementation());
    matFile_.put("Hi_", Hi_.toImplementation());
    matFile_.put("Hi_inverse", Hi_inverse_.toImplementation());
    matFile_.put("G", G_.toImplementation());
    matFile_.put("gv", gv_.toImplementation());

    matFile_.close();
#endif
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void GNRiccatiSolver<MANIFOLD, CONTROL_DIM>::initializeAndAllocate()
{
    // do nothing
}

}  // namespace optcon
}  // namespace ct
