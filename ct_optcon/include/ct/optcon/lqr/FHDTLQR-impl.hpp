/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
FHDTLQR<STATE_DIM, CONTROL_DIM, SCALAR>::FHDTLQR(
    std::shared_ptr<CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>> costFunction)
    : costFunction_(costFunction)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
FHDTLQR<STATE_DIM, CONTROL_DIM, SCALAR>::~FHDTLQR()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void FHDTLQR<STATE_DIM, CONTROL_DIM, SCALAR>::designController(const state_vector_array_t& x_trajectory,
    const control_vector_array_t& u_trajectory,
    const state_matrix_array_t& A,
    const control_gain_matrix_array_t& B,
    SCALAR dt,
    control_feedback_array_t& K,
    bool performNumericalChecks)
{
    size_t N = x_trajectory.size() - 1;
    solve(x_trajectory, u_trajectory, A, B, N, dt, K, performNumericalChecks);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void FHDTLQR<STATE_DIM, CONTROL_DIM, SCALAR>::designController(const state_vector_array_t& x_trajectory,
    const control_vector_array_t& u_trajectory,
    std::shared_ptr<core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>> derivatives,
    SCALAR dt,
    control_feedback_array_t& K,
    bool performNumericalChecks)
{
    size_t N = x_trajectory.size() - 1;

    state_matrix_array_t A;
    control_gain_matrix_array_t B;

    linearizeModel(x_trajectory, u_trajectory, N, dt, derivatives, A, B);

    solve(x_trajectory, u_trajectory, A, B, N, dt, K, performNumericalChecks);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void FHDTLQR<STATE_DIM, CONTROL_DIM, SCALAR>::linearizeModel(const state_vector_array_t& x_trajectory,
    const control_vector_array_t& u_trajectory,
    size_t N,
    SCALAR dt,
    std::shared_ptr<core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& derivatives,
    state_matrix_array_t& A,
    control_gain_matrix_array_t& B)
{
    A.resize(N);
    B.resize(N);

    for (int i = 0; i < N; i++)
    {
        A[i] = state_matrix_t::Identity() + dt * derivatives->getDerivativeState(x_trajectory[i], u_trajectory[i]);
        B[i] = dt * derivatives->getDerivativeControl(x_trajectory[i], u_trajectory[i]);
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void FHDTLQR<STATE_DIM, CONTROL_DIM, SCALAR>::solve(const state_vector_array_t& x_trajectory,
    const control_vector_array_t& u_trajectory,
    const state_matrix_array_t& A,
    const control_gain_matrix_array_t& B,
    size_t N,
    SCALAR dt,
    control_feedback_array_t& K,
    bool performNumericalChecks)
{
    if (x_trajectory.size() != N + 1)
    {
        throw std::runtime_error("State trajectory does not have correct length. State trajectory length: " +
                                 std::to_string(x_trajectory.size()) + ", should be: " + std::to_string(N + 1));
    }
    if (u_trajectory.size() != N)
    {
        throw std::runtime_error("Input trajectory does not have correct length. Input trajectory length: " +
                                 std::to_string(u_trajectory.size()) + ", should be: " + std::to_string(N));
    }
    if (A.size() != N)
    {
        throw std::runtime_error("Linearization A does not have correct length. Linearization length: " +
                                 std::to_string(A.size()) + ", should be: " + std::to_string(N));
    }
    if (B.size() != N)
    {
        throw std::runtime_error("Linearization B does not have correct length. Linearization length: " +
                                 std::to_string(B.size()) + ", should be: " + std::to_string(N));
    }

    K.resize(N);

    // initialize cost-to-go
    costFunction_->setCurrentStateAndControl(x_trajectory[N], control_vector_t::Zero(), dt * N);
    state_matrix_t P = costFunction_->stateSecondDerivativeTerminal();

    if (performNumericalChecks)
    {
        Eigen::Matrix<SCALAR, -1, 1> realEigVals = P.eigenvalues().real();
        if (realEigVals.minCoeff() < 0.0)
        {
            std::cout << "P: " << std::endl << P;
            throw std::runtime_error("[LQR] Q is not positive semi-definite.");
        }
    }

    for (int i = N - 1; i >= 0; i--)
    {
        costFunction_->setCurrentStateAndControl(x_trajectory[i], u_trajectory[i], i * dt);

        state_matrix_t Q = costFunction_->stateSecondDerivativeIntermediate() * dt;

        control_matrix_t R = costFunction_->controlSecondDerivativeIntermediate() * dt;

        if (performNumericalChecks)
        {
            if (Q.minCoeff() < 0.0)
            {
                std::cout << "Q: " << std::endl << Q;
                throw std::runtime_error("[LQR] Q contains negative entries.");
            }
            if (R.minCoeff() < 0.0)
            {
                std::cout << "R: " << std::endl << R;
                throw std::runtime_error("[LQR] R contains negative entries.");
            }
            if (R.diagonal().minCoeff() <= 0.0)
            {
                std::cout << "R: " << std::endl << R;
                throw std::runtime_error("[LQR] R contains zero entries on the diagonal.");
            }
            if (!Q.isDiagonal())
            {
                std::cout << "[LQR] Warning: Q is not diagonal." << std::endl;
            }
            if (!R.isDiagonal())
            {
                std::cout << "[LQR] Warning: R is not diagonal." << std::endl;
            }
        }

        //ricattiEq_.iterateNaive(Q, R, A, B, P, K[i]);
        ricattiEq_.iterateRobust(Q, R, A[i], B[i], P, K[i]);
    }
}

}  // namespace optcon
}  // namespace ct
