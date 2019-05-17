/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {
namespace example {


/*!
 *
 * We named this system after Prof. Moritz Diehl from the University of Freiburg, who gave it to us as a simple 1-dimensional test system.
 * The system dynamics are dx/dt = (1+x)x + u + 0.1
 *
 */
//! Dynamics class for the Diehl system
class DiehlSystem : public core::ControlledSystem<1, 1>
{
public:
    static const int state_dim = 1;
    static const int control_dim = 1;

    DiehlSystem() : core::ControlledSystem<1, 1>(core::SYSTEM_TYPE::SECOND_ORDER) {}
    void computeControlledDynamics(const core::StateVector<1>& state,
        const core::Time& t,
        const core::ControlVector<1>& control,
        core::StateVector<1>& derivative) override
    {
        derivative(0) = (1.0 + state(0)) * state(0) + control(0) + 0.1;
    }

    DiehlSystem* clone() const override { return new DiehlSystem(); };
};

//! Linear system class for the Diehl system
class DiehlSystemLinear : public core::LinearSystem<1, 1>
{
public:
    static const int state_dim = 1;
    static const int control_dim = 1;

    state_matrix_t A_;
    state_control_matrix_t B_;


    const state_matrix_t& getDerivativeState(const core::StateVector<1>& x,
        const core::ControlVector<1>& u,
        const double t = 0.0) override
    {
        A_ << 1 + 2 * x(0);
        return A_;
    }

    const state_control_matrix_t& getDerivativeControl(const core::StateVector<1>& x,
        const core::ControlVector<1>& u,
        const double t = 0.0) override
    {
        B_ << 1;
        return B_;
    }

    DiehlSystemLinear* clone() const override { return new DiehlSystemLinear(); }
};


//! create a cost function of appropriate Dimensions for the Diehl system
std::shared_ptr<CostFunctionQuadratic<1, 1>> createDiehlCostFunction(const core::StateVector<1>& x_final)
{
    Eigen::Matrix<double, 1, 1> Q;
    Q << 1.0;

    Eigen::Matrix<double, 1, 1> R;
    R << 1.0;

    Eigen::Matrix<double, 1, 1> x_nominal = x_final;
    Eigen::Matrix<double, 1, 1> u_nominal;
    u_nominal.setConstant(-0.1);  //Eigen::Matrix<double, 1, 1>::Zero();

    Eigen::Matrix<double, 1, 1> Q_final;
    Q_final << 10.0;

    std::shared_ptr<CostFunctionQuadratic<1, 1>> quadraticCostFunction(
        new CostFunctionQuadraticSimple<1, 1>(Q, R, x_nominal, u_nominal, x_final, Q_final));

    return quadraticCostFunction;
}
}
}
}
