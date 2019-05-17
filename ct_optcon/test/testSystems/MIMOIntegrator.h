/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {
namespace example {


//! Dynamics class for the GNMS unit test

template <size_t state_dim, size_t control_dim>
class MIMOIntegrator : public core::ControlledSystem<state_dim, control_dim>
{
public:
    MIMOIntegrator() : core::ControlledSystem<state_dim, control_dim>(core::SYSTEM_TYPE::SECOND_ORDER) {}
    void computeControlledDynamics(const core::StateVector<state_dim>& state,
        const core::Time& t,
        const core::ControlVector<control_dim>& control,
        core::StateVector<state_dim>& derivative) override
    {
        for (size_t i = 0; i < state_dim; i++)
        {
            derivative(i) = state(i);
        }
        for (size_t j = 0; j < control_dim; j++)
        {
            derivative(j) += control(j);
        }
    }

    MIMOIntegrator* clone() const override { return new MIMOIntegrator(); };
};

//! Linear system class for the GNMS unit test
template <size_t state_dim, size_t control_dim>
class MIMOIntegratorLinear : public core::LinearSystem<state_dim, control_dim>
{
public:
    using BASE = core::LinearSystem<state_dim, control_dim>;
    typedef typename BASE::state_matrix_t state_matrix_t;
    typedef typename BASE::state_control_matrix_t state_control_matrix_t;

    state_matrix_t A_;
    state_control_matrix_t B_;

    MIMOIntegratorLinear()
    {
        A_.setIdentity();
        B_.setIdentity();
    }


    const state_matrix_t& getDerivativeState(const core::StateVector<state_dim>& x,
        const core::ControlVector<control_dim>& u,
        const double t = 0.0) override
    {
        return A_;
    }

    const state_control_matrix_t& getDerivativeControl(const core::StateVector<state_dim>& x,
        const core::ControlVector<control_dim>& u,
        const double t = 0.0) override
    {
        return B_;
    }

    MIMOIntegratorLinear* clone() const override { return new MIMOIntegratorLinear(); };
};


template <size_t state_dim, size_t control_dim>
std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> createMIMOIntegratorCostFunction(
    const core::StateVector<state_dim>& x_final)
{
    Eigen::Matrix<double, state_dim, state_dim> Q;
    Q.setIdentity();

    Eigen::Matrix<double, control_dim, control_dim> R;
    R.setIdentity();

    Eigen::Matrix<double, state_dim, 1> x_nominal = x_final;
    Eigen::Matrix<double, control_dim, 1> u_nominal;
    u_nominal.setZero();

    Eigen::Matrix<double, state_dim, state_dim> Q_final;
    Q_final.setIdentity();

    std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> quadraticCostFunction(
        new CostFunctionQuadraticSimple<state_dim, control_dim>(Q, R, x_nominal, u_nominal, x_final, Q_final));

    return quadraticCostFunction;
}
}
}
}
