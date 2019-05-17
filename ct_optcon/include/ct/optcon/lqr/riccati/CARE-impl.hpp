/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM>
CARE<STATE_DIM, CONTROL_DIM>::CARE()
{
    // we have to find the optimal work size of schur reordering
    schur_matrix_t T;
    schur_matrix_t U;

    int SELECT[2 * STATE_DIM];
    int N = 2 * STATE_DIM;
    double WR[T.ColsAtCompileTime];
    double WI[T.ColsAtCompileTime];
    int MS;
    double S;
    double SEP;
    double WORKDUMMY[1];
    int LWORK = -1;
    int IWORKQUERY[1];
    int LIWORK = -1;
    int INFO = 0;
    int TCols = schur_matrix_t::ColsAtCompileTime;

#ifdef CT_USE_LAPACK
    dtrsen_("N", "V", &SELECT[0], &TCols, T.data(), &N, U.data(), &N, &WR[0], &WI[0], &MS, &S, &SEP, WORKDUMMY, &LWORK,
        &IWORKQUERY[0], &LIWORK, &INFO);

    LWORK_ = WORKDUMMY[0] + 32;
    LIWORK_ = IWORKQUERY[0] + 32;

    WORK_.resize(LWORK_);
    IWORK_.resize(LIWORK_);

    if (INFO != 0)
    {
        std::cout << "Lapack invocation of dtrsen failed!" << std::endl;
        exit(-1);
    }
#endif
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
bool CARE<STATE_DIM, CONTROL_DIM>::solve(const state_matrix_t& Q,
    const control_matrix_t& R,
    const state_matrix_t& A,
    const control_gain_matrix_t& B,
    state_matrix_t& P,
    bool RisDiagonal,
    control_matrix_t& R_inverse,
    bool useIterativeSolver)
{
    if (RisDiagonal)
    {
        R_inverse.setZero();
        R_inverse.diagonal().noalias() = R.diagonal().cwiseInverse();
    }
    else
    {
        R_inverse.noalias() = R.inverse();
    }

    schur_matrix_t M;
    M << A, -B * R_inverse * B.transpose(), -Q, -A.transpose();

    if (useIterativeSolver)
        return solveSchurIterative(M, P);
    else
        return solveSchurDirect(M, P);
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
typename CARE<STATE_DIM, CONTROL_DIM>::state_matrix_t CARE<STATE_DIM, CONTROL_DIM>::computeSteadyStateRiccatiMatrix(
    const state_matrix_t& Q,
    const control_matrix_t& R,
    const state_matrix_t& A,
    const control_gain_matrix_t& B,
    const bool RisDiagonal,
    const bool useIterativeSolver)
{
    state_matrix_t P;
    control_matrix_t Rinv;

    solve(Q, R, A, B, P, RisDiagonal, Rinv, useIterativeSolver);

    return P;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
bool CARE<STATE_DIM, CONTROL_DIM>::solveSchurIterative(const schur_matrix_t& M,
    state_matrix_t& P,
    double epsilon,
    int maxIterations)
{
    bool converged = false;

    schur_matrix_t Mlocal = M;

    int iterations = 0;
    while (!converged)
    {
        if (iterations > maxIterations)
            return false;

        schur_matrix_t Mdiff = Mlocal - Mlocal.inverse();

        schur_matrix_t Mnew = Mlocal - 0.5 * Mdiff;

        converged = Mnew.isApprox(Mlocal, epsilon);

        Mlocal = Mnew;

        iterations++;
    }

    /* break down W and extract W11 W12 W21 W22  (what is the size of these?) */
    state_matrix_t M11(Mlocal.template block<STATE_DIM, STATE_DIM>(0, 0));
    state_matrix_t M12(Mlocal.template block<STATE_DIM, STATE_DIM>(0, STATE_DIM));
    state_matrix_t M21(Mlocal.template block<STATE_DIM, STATE_DIM>(STATE_DIM, 0));
    state_matrix_t M22(Mlocal.template block<STATE_DIM, STATE_DIM>(STATE_DIM, STATE_DIM));

    /* find M and N using the elements of W	 */
    factor_matrix_t U;
    factor_matrix_t V;

    U.template block<STATE_DIM, STATE_DIM>(0, 0) = M12;
    U.template block<STATE_DIM, STATE_DIM>(STATE_DIM, 0) = M22 + state_matrix_t::Identity();

    V.template block<STATE_DIM, STATE_DIM>(0, 0) = M11 + state_matrix_t::Identity();
    V.template block<STATE_DIM, STATE_DIM>(STATE_DIM, 0) = M21;


    /* Solve for S from the equation   MS=N */
    FullPivLU_.compute(U);

    P = FullPivLU_.solve(-V);

    return true;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
bool CARE<STATE_DIM, CONTROL_DIM>::solveSchurDirect(const schur_matrix_t& M, state_matrix_t& P)
{
#ifdef CT_USE_LAPACK
    const bool computeU = true;
    schur_.compute(M, computeU);

    if (schur_.info() != Eigen::Success)
    {
        throw std::runtime_error(
            "LQR Schur computation failed. Most likely problem is set up wrongly or not solvable.");
    }

    schur_matrix_t U(schur_.matrixU());
    schur_matrix_t T(schur_.matrixT());

    int SELECT[2 * STATE_DIM];
    double WR[2 * STATE_DIM];
    double WI[2 * STATE_DIM];
    int MS;
    double S;
    double SEP;
    int INFO = 0;
    int N = 2 * STATE_DIM;

    for (size_t i = 0; i < 2 * STATE_DIM; i++)
    {
        // check if last row or eigenvalue is complex (2x2 block)
        if (i == (2 * STATE_DIM - 1) || std::abs(T(i + 1, i)) < 1e-12)
        {
            SELECT[i] = static_cast<int>(T(i, i) < 0);
        }
        else
        {
            // we have a complex block
            SELECT[i] = static_cast<int>((T(i, i) + T(i + 1, i + 1)) / 2.0 < 0);
            SELECT[i + 1] = SELECT[i];
            i++;
        }
    }

    dtrsen_("N", "V", &SELECT[0], &N, T.data(), &N, U.data(), &N, &WR[0], &WI[0], &MS, &S, &SEP, WORK_.data(), &LWORK_,
        IWORK_.data(), &LIWORK_, &INFO);

    const state_matrix_t& U11 = U.template block<STATE_DIM, STATE_DIM>(0, 0);
    const state_matrix_t& U21 = U.template block<STATE_DIM, STATE_DIM>(STATE_DIM, 0);

    // solve here for better numerical properties
    P.noalias() = U21 * U11.inverse();

    if (INFO != 0)
    {
        return false;
    }

    return true;
#else
    throw std::runtime_error(
        "solveSchurDirect() in CARE can only be used if the lapack library is installed on your system.");
#endif
}


}  // namespace optcon
}  // namespace ct
