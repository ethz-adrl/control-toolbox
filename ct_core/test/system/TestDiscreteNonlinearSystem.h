/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/core.h>

namespace ct {
namespace core {
namespace tpl {

template <typename SCALAR>
class TestDiscreteNonlinearSystem final : public ControlledSystem<EuclideanState<2, SCALAR>, 1, DISCRETE_TIME>
{
public:
    static const size_t STATE_DIM = 2;
    static const size_t CONTROL_DIM = 1;

    using state_vector_t = EuclideanState<STATE_DIM, SCALAR>;

    using Base = ControlledSystem<state_vector_t, CONTROL_DIM, DISCRETE_TIME>;
    typedef typename Base::control_vector_t control_vector_t;
    typedef typename Base::Time_t Time_t;

    TestDiscreteNonlinearSystem() = delete;

    // constructor directly using frequency and damping coefficients
    TestDiscreteNonlinearSystem(SCALAR rate,
        std::shared_ptr<Controller<state_vector_t, CONTROL_DIM, DISCRETE_TIME>> controller = nullptr)
        : Base(controller, SYSTEM_TYPE::GENERAL), rate_(rate)
    {
    }

    //copy constructor
    TestDiscreteNonlinearSystem(const TestDiscreteNonlinearSystem& arg) : Base(arg), rate_(arg.rate_) {}
    virtual ~TestDiscreteNonlinearSystem() {}
    TestDiscreteNonlinearSystem* clone() const override { return new TestDiscreteNonlinearSystem(*this); }
    void computeControlledDynamics(const state_vector_t& state,
        const Time_t& n,
        const control_vector_t& control,
        typename state_vector_t::Tangent& dx) override
    {
        // this is pretty much random
        dx(0) = rate_ * state(0) * control(0);
        dx(1) = state(0) * state(1) * state(1);
    }

    // analytic matrices which fulfil x_{n+1} = A*x_n + B*u_n
    // note that computeControlledDynamics only computes the resdidual dx
    StateMatrix<STATE_DIM> get_A_analytic(state_vector_t& x, const control_vector_t& u) const
    {
        StateMatrix<STATE_DIM> A_analytic;
        A_analytic << 1 + rate_ * u(0), 0.0, x(1) * x(1), 1 + 2.0 * x(0) * x(1);
        return A_analytic;
    }
    StateControlMatrix<STATE_DIM, CONTROL_DIM> get_B_analytic(state_vector_t& x, const control_vector_t& u) const
    {
        StateControlMatrix<STATE_DIM, CONTROL_DIM> B_analytic;
        B_analytic << rate_ * x(0), 0.0;
        return B_analytic;
    }

private:
    SCALAR rate_;
};
}  // namespace tpl

typedef tpl::TestDiscreteNonlinearSystem<double> TestDiscreteNonlinearSystem;

}  // namespace core
}  // namespace ct
