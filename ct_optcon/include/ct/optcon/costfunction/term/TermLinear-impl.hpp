/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermLinear<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermLinear(const core::StateVector<STATE_DIM, SCALAR_EVAL> a,
    core::ControlVector<CONTROL_DIM, SCALAR_EVAL> b,
    const SCALAR_EVAL c)
    : a_(a), b_(b), c_(c)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermLinear<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermLinear(const TermLinear& arg)
    : TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>(arg), a_(arg.a_), b_(arg.b_), c_(arg.c_)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermLinear<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>*
TermLinear<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::clone() const
{
    return new TermLinear<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermLinear<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermLinear()
{
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermLinear<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::~TermLinear()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
SCALAR TermLinear<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
    const Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
    const SCALAR& t)
{
    return evalLocal<SCALAR>(x, u, t);
}

#ifdef CPPADCG
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
ct::core::ADCGScalar TermLinear<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::evaluateCppadCg(
    const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
    const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
    ct::core::ADCGScalar t)
{
    return evalLocal<ct::core::ADCGScalar>(x, u, t);
}
#endif

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
core::StateVector<STATE_DIM, SCALAR_EVAL> TermLinear<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    return a_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermLinear<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::state_matrix_t
TermLinear<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateSecondDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    return state_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
core::ControlVector<CONTROL_DIM, SCALAR_EVAL>
TermLinear<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::controlDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    return b_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermLinear<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::control_matrix_t
TermLinear<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::controlSecondDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    return control_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermLinear<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::control_state_matrix_t
TermLinear<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateControlDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    return control_state_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermLinear<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::loadConfigFile(const std::string& filename,
    const std::string& termName,
    bool verbose)
{
    // read in the file and put the valus in a_ and b_
    loadMatrixCF(filename, "a", a_, termName);
    loadMatrixCF(filename, "b", b_, termName);
    if (verbose)
    {
        std::cout << "Read a as a= \n" << a_ << std::endl;
        std::cout << "Read b as b= \n" << b_ << std::endl;
    }
}
}  // namespace optcon
}  // namespace ct
