/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <cmath>
#include <memory>
#include <iostream>

namespace ct {
namespace core {


namespace tpl {

template <typename SCALAR>
class TestTimeParameterizedLinearSystem : public ControlledSystem<3, 2, SCALAR>
{
public:
    typedef ControlledSystem<3, 2, SCALAR> Base;
    static const size_t STATE_DIM = 3;
    static const size_t CONTROL_DIM = 2;

    typedef StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef ControlVector<CONTROL_DIM, SCALAR> control_vector_t;
    typedef typename Base::time_t time_t;

    typedef StateMatrix<2, SCALAR> state_matrix_t;                    //!< state Jacobian type
    typedef StateControlMatrix<2, 1, SCALAR> state_control_matrix_t;  //!< input Jacobian type

    TestTimeParameterizedLinearSystem() = delete;

    // constructor directly using A and B
    TestTimeParameterizedLinearSystem(state_matrix_t& A,
        state_control_matrix_t& B,
        std::shared_ptr<Controller<3, 2, SCALAR>> controller = nullptr)
        : Base(controller, SYSTEM_TYPE::GENERAL), A_(A), B_(B)
    {
    }

    //copy constructor
    TestTimeParameterizedLinearSystem(const TestTimeParameterizedLinearSystem& arg) : Base(arg), A_(arg.A_), B_(arg.B_)
    {
    }

    virtual ~TestTimeParameterizedLinearSystem() {}
    TestTimeParameterizedLinearSystem* clone() const override { return new TestTimeParameterizedLinearSystem(*this); }
    virtual void computeControlledDynamics(const StateVector<STATE_DIM, SCALAR>& state,
        const time_t& t,
        const ControlVector<CONTROL_DIM, SCALAR>& control,
        StateVector<STATE_DIM, SCALAR>& derivative) override
    {
        derivative.head(2) = control(1) * (A_ * state.head(2) + B_ * control(0));
        derivative(2) = control(1);
    }

private:
    state_matrix_t A_;
    state_control_matrix_t B_;
};


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

template <typename SCALAR>
class TestDiscreteLinearSystem : public DiscreteControlledSystem<2, 1, SCALAR>
{
public:
    typedef DiscreteControlledSystem<2, 1, SCALAR> Base;
    static const size_t STATE_DIM = 2;
    static const size_t CONTROL_DIM = 1;

    typedef typename Base::state_vector_t state_vector_t;
    typedef typename Base::control_vector_t control_vector_t;
    typedef typename Base::time_t time_t;

    typedef StateMatrix<STATE_DIM, SCALAR> state_matrix_t;                              //!< state Jacobian type
    typedef StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR> state_control_matrix_t;  //!< input Jacobian type

    TestDiscreteLinearSystem() = delete;

    // constructor directly using A and B
    TestDiscreteLinearSystem(state_matrix_t A,
        state_control_matrix_t B,
        std::shared_ptr<DiscreteController<2, 1, SCALAR>> controller = nullptr)
        : Base(controller, SYSTEM_TYPE::GENERAL), A_(A), B_(B)
    {
    }

    //copy constructor
    TestDiscreteLinearSystem(const TestDiscreteLinearSystem& arg) : Base(arg), A_(arg.A_), B_(arg.B_) {}
    virtual ~TestDiscreteLinearSystem() {}
    TestDiscreteLinearSystem* clone() const override { return new TestDiscreteLinearSystem(*this); }
    virtual void propagateControlledDynamics(const state_vector_t& state,
        const time_t n,
        const control_vector_t& control,
        state_vector_t& stateNext) override
    {
        stateNext = A_ * state + B_ * control;
    }

private:
    state_matrix_t A_;
    state_control_matrix_t B_;
};

template <typename SCALAR>
class TestSystems : public DiscreteControlledSystem<2, 1, SCALAR>
{
public:
    typedef DiscreteControlledSystem<2, 1, SCALAR> Base;
    static const size_t STATE_DIM = 2;
    static const size_t CONTROL_DIM = 1;

    typedef typename Base::state_vector_t state_vector_t;
    typedef typename Base::control_vector_t control_vector_t;
    typedef typename Base::time_t time_t;

    TestSystems() = delete;

    // constructor directly using frequency and damping coefficients
    TestSystems(SCALAR rate, std::shared_ptr<DiscreteController<2, 1, SCALAR>> controller = nullptr)
        : Base(controller, SYSTEM_TYPE::GENERAL), rate_(rate)
    {
    }

    //copy constructor
    TestSystems(const TestSystems& arg) : Base(arg), rate_(arg.rate_) {}
    virtual ~TestSystems() {}
    TestSystems* clone() const override { return new TestSystems(*this); }
    virtual void propagateControlledDynamics(const state_vector_t& state,
        const time_t n,
        const control_vector_t& control,
        state_vector_t& stateNext) override
    {
        // this is pretty much random
        stateNext(0) = state(0) + rate_ * state(0) * control(0);
        stateNext(1) = state(0) * state(1) * state(1);
    }

private:
    SCALAR rate_;
};
}  // namespace tpl

typedef tpl::TestSystems<double> TestDiscreteNonlinearSystem;
typedef tpl::TestDiscreteLinearSystem<double> TestDiscreteLinearSystem;
typedef tpl::TestLinearSystem<double> TestLinearSystem;
typedef tpl::TestTimeParameterizedLinearSystem<double> TestTimeParameterizedLinearSystem;


}  // namespace core
}  // namespace ct
