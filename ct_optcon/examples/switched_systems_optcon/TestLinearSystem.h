/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {


namespace tpl {

template <typename SCALAR>
class TestLinearSystem : public ControlledSystem<2, 1, SCALAR>
{
public:
    typedef ControlledSystem<2, 1, SCALAR> Base;
    static const size_t STATE_DIM = 2;
    static const size_t CONTROL_DIM = 1;

    typedef StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef ControlVector<CONTROL_DIM, SCALAR> control_vector_t;
    typedef typename Base::time_t time_t;

    typedef StateMatrix<STATE_DIM, SCALAR> state_matrix_t;                              //!< state Jacobian type
    typedef StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR> state_control_matrix_t;  //!< input Jacobian type

    TestLinearSystem() = delete;

    // constructor directly using A and B
    TestLinearSystem(state_matrix_t A,
        state_control_matrix_t B,
        std::shared_ptr<Controller<2, 1, SCALAR>> controller = nullptr)
        : Base(controller, SYSTEM_TYPE::GENERAL), A_(A), B_(B)
    {
    }

    //copy constructor
    TestLinearSystem(const TestLinearSystem& arg) : Base(arg), A_(arg.A_), B_(arg.B_) {}
    virtual ~TestLinearSystem() {}
    TestLinearSystem* clone() const override { return new TestLinearSystem(*this); }
    virtual void computeControlledDynamics(const StateVector<STATE_DIM, SCALAR>& state,
        const time_t& t,
        const ControlVector<CONTROL_DIM, SCALAR>& control,
        StateVector<STATE_DIM, SCALAR>& derivative) override
    {
        derivative = A_ * state + B_ * control;
    }

private:
    state_matrix_t A_;
    state_control_matrix_t B_;
};

}  // namespace tpl

typedef tpl::TestLinearSystem<double> TestLinearSystem;

}  // namespace core
}  // namespace ct
