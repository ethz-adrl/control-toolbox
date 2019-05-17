/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {
namespace example {


using namespace ct::core;
using namespace ct::optcon;

using std::shared_ptr;

const size_t state_dim = 2;    // position, velocity
const size_t control_dim = 1;  // force

const double kStiffness = 0.1;


namespace tpl {

template <typename SCALAR = double>
class LinearOscillator : public ControlledSystem<state_dim, control_dim, SCALAR>
{
public:
    LinearOscillator() : ControlledSystem<state_dim, control_dim, SCALAR>(SYSTEM_TYPE::GENERAL) {}
    void computeControlledDynamics(const StateVector<state_dim, SCALAR>& state,
        const SCALAR& t,
        const ControlVector<control_dim, SCALAR>& control,
        StateVector<state_dim, SCALAR>& derivative) override
    {
        derivative(0) = state(1);
        derivative(1) = control(0) - kStiffness * state(0);  // mass is 1 kg
    }

    LinearOscillator<SCALAR>* clone() const override { return new LinearOscillator<SCALAR>(); };
};


template <typename SCALAR = double>
class LinearOscillatorLinear : public LinearSystem<state_dim, control_dim, SCALAR>
{
public:
    typedef core::StateMatrix<state_dim, SCALAR> state_matrix_t;
    typedef core::StateControlMatrix<state_dim, control_dim, SCALAR> state_control_matrix_t;

    state_matrix_t A_;
    state_control_matrix_t B_;

    const state_matrix_t& getDerivativeState(const StateVector<state_dim, SCALAR>& x,
        const ControlVector<control_dim, SCALAR>& u,
        const SCALAR t = 0.0) override
    {
        A_ << 0, 1, -kStiffness, 0;
        return A_;
    }

    const state_control_matrix_t& getDerivativeControl(const StateVector<state_dim, SCALAR>& x,
        const ControlVector<control_dim, SCALAR>& u,
        const SCALAR t = 0.0) override
    {
        B_ << 0, 1;
        return B_;
    }

    LinearOscillatorLinear<SCALAR>* clone() const override { return new LinearOscillatorLinear<SCALAR>(); };
};


template <typename SCALAR = double>
std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim, SCALAR>> createCostFunctionLinearOscillator(
    Eigen::Matrix<SCALAR, 2, 1>& x_final)
{
    Eigen::Matrix<SCALAR, 2, 2> Q;
    Q << 0, 0, 0, 1;

    Eigen::Matrix<SCALAR, 1, 1> R;
    R << 100;

    ct::core::StateVector<2> x_nominal = ct::core::StateVector<2>::Zero();
    ct::core::ControlVector<1> u_nominal = ct::core::ControlVector<1>::Zero();

    Eigen::Matrix<SCALAR, 2, 2> Q_final;
    Q_final << 1000, 0, 0, 1000;

    std::shared_ptr<TermQuadratic<state_dim, control_dim>> termIntermediate(
        new TermQuadratic<state_dim, control_dim>(Q, R, x_nominal, u_nominal));
    std::shared_ptr<TermQuadratic<state_dim, control_dim>> termFinal(
        new TermQuadratic<state_dim, control_dim>(Q_final, R, x_final, u_nominal));

    std::shared_ptr<CostFunctionAnalytical<state_dim, control_dim>> quadraticCostFunction(
        new CostFunctionAnalytical<state_dim, control_dim>);
    quadraticCostFunction->addIntermediateTerm(termIntermediate);
    quadraticCostFunction->addFinalTerm(termFinal);

    return quadraticCostFunction;
}

}  // namespace tpl

typedef tpl::LinearOscillator<double> LinearOscillator;
typedef tpl::LinearOscillatorLinear<double> LinearOscillatorLinear;

}  //example
}  //optcon
}  //ct
