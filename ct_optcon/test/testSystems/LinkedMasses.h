/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * This header implements example systems from the HPIPM library as CT systems
 *
 * \warning This example system is not intuitive. For a better examples, visit the tutorial.
 */

#pragma once

using namespace ct::core;


void dmcopy(int row, int col, double* A, int lda, double* B, int ldb)
{
    int i, j;
    for (j = 0; j < col; j++)
    {
        for (i = 0; i < row; i++)
        {
            B[i + j * ldb] = A[i + j * lda];
        }
    }
}

class LinkedMasses : public LinearSystem<8, 3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const int state_dim = 8;
    static const int control_dim = 3;
    static const int pp = state_dim / 2;  // number of masses

    LinkedMasses()
    {
        A_.setZero();
        B_.setZero();


        Eigen::Matrix<double, pp, pp> TEigen;
        TEigen.setZero();

        double* T = TEigen.data();
        int ii;
        for (ii = 0; ii < pp; ii++)
            T[ii * (pp + 1)] = -2;
        for (ii = 0; ii < pp - 1; ii++)
            T[ii * (pp + 1) + 1] = 1;
        for (ii = 1; ii < pp; ii++)
            T[ii * (pp + 1) - 1] = 1;

        Eigen::Matrix<double, pp, pp> ZEigen;
        ZEigen.setZero();
        double* Z = ZEigen.data();

        Eigen::Matrix<double, pp, pp> IEigen;
        IEigen.setIdentity();
        double* I = IEigen.data();

        double* Ac = A_.data();
        dmcopy(pp, pp, Z, pp, Ac, state_dim);
        dmcopy(pp, pp, T, pp, Ac + pp, state_dim);
        dmcopy(pp, pp, I, pp, Ac + pp * state_dim, state_dim);
        dmcopy(pp, pp, Z, pp, Ac + pp * (state_dim + 1), state_dim);

        Eigen::Matrix<double, control_dim, control_dim> InuEigen;
        InuEigen.setIdentity();
        double* Inu = InuEigen.data();

        double* Bc = B_.data();
        dmcopy(control_dim, control_dim, Inu, control_dim, Bc + pp, state_dim);
    }


    const state_matrix_t& getDerivativeState(const StateVector<state_dim>& x,
        const ControlVector<control_dim>& u,
        const double t = 0.0) override
    {
        return A_;
    }

    const state_control_matrix_t& getDerivativeControl(const StateVector<state_dim>& x,
        const ControlVector<control_dim>& u,
        const double t = 0.0) override
    {
        return B_;
    }

    LinkedMasses* clone() const override { return new LinkedMasses(); };
private:
    state_matrix_t A_;
    state_control_matrix_t B_;
};


class LinkedMasses2 : public ControlledSystem<8, 3>
{
public:
    static const int state_dim = 8;
    static const int control_dim = 3;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LinkedMasses2()
    {
        A_.setZero();
        B_.setZero();
        b_.setZero();

        b_ << 0.145798, 0.150018, 0.150018, 0.145798, 0.245798, 0.200018, 0.200018, 0.245798;

        static const int pp = state_dim / 2;  // number of masses

        Eigen::Matrix<double, pp, pp> TEigen;
        TEigen.setZero();

        double* T = TEigen.data();
        int ii;
        for (ii = 0; ii < pp; ii++)
            T[ii * (pp + 1)] = -2;
        for (ii = 0; ii < pp - 1; ii++)
            T[ii * (pp + 1) + 1] = 1;
        for (ii = 1; ii < pp; ii++)
            T[ii * (pp + 1) - 1] = 1;

        Eigen::Matrix<double, pp, pp> ZEigen;
        ZEigen.setZero();
        double* Z = ZEigen.data();

        Eigen::Matrix<double, pp, pp> IEigen;
        IEigen.setIdentity();
        double* I = IEigen.data();

        double* Ac = A_.data();
        dmcopy(pp, pp, Z, pp, Ac, state_dim);
        dmcopy(pp, pp, T, pp, Ac + pp, state_dim);
        dmcopy(pp, pp, I, pp, Ac + pp * state_dim, state_dim);
        dmcopy(pp, pp, Z, pp, Ac + pp * (state_dim + 1), state_dim);

        Eigen::Matrix<double, control_dim, control_dim> InuEigen;
        InuEigen.setIdentity();
        double* Inu = InuEigen.data();

        double* Bc = B_.data();
        dmcopy(control_dim, control_dim, Inu, control_dim, Bc + pp, state_dim);
    }

    LinkedMasses2* clone() const override { return new LinkedMasses2(); };
    void computeControlledDynamics(const ct::core::StateVector<state_dim>& state,
        const double& t,
        const ct::core::ControlVector<control_dim>& control,
        ct::core::StateVector<state_dim>& derivative) override
    {
        derivative = A_ * state + B_ * control + b_;
    }

private:
    ct::core::StateMatrix<state_dim> A_;
    ct::core::StateControlMatrix<state_dim, control_dim> B_;
    ct::core::StateVector<state_dim> b_;
};
