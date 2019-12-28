/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermMixed(const control_state_matrix_t& P) : P_(P)
{
    x_ref_.setZero();  // default values
    u_ref_.setZero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermMixed()
{
    P_.setZero();  // default values
    x_ref_.setZero();
    u_ref_.setZero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermMixed(const control_state_matrix_t& P,
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x_ref,
    core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u_ref)
    : P_(P), x_ref_(x_ref), u_ref_(u_ref)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermMixed(
    const TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>& arg)
    : TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>(arg), P_(arg.P_), x_ref_(arg.x_ref_), u_ref_(arg.u_ref_)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>* TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::clone()
    const
{
    return new TermMixed(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::~TermMixed()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::setWeights(const control_state_matrix_double_t& P)
{
    P_ = P.template cast<SCALAR_EVAL>();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::setStateAndControlReference(
    const core::StateVector<STATE_DIM>& x_ref,
    const core::ControlVector<CONTROL_DIM>& u_ref)
{
    x_ref_ = x_ref.template cast<SCALAR_EVAL>();
    u_ref_ = u_ref.template cast<SCALAR_EVAL>();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
SCALAR TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
    const Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
    const SCALAR& t)
{
    return evalLocal<SCALAR>(x, u, t);
}

#ifdef CPPADCG
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
ct::core::ADCGScalar TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::evaluateCppadCg(
    const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
    const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
    ct::core::ADCGScalar t)
{
    return evalLocal<ct::core::ADCGScalar>(x, u, t);
}
#endif

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
ct::core::StateVector<STATE_DIM, SCALAR_EVAL> TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateDerivative(
    const ct::core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    core::ControlVector<CONTROL_DIM, SCALAR_EVAL> uDiff = (u - u_ref_);

    return P_.transpose() * uDiff;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::state_matrix_t
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateSecondDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    return state_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
core::ControlVector<CONTROL_DIM, SCALAR_EVAL> TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::controlDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    core::StateVector<STATE_DIM, SCALAR_EVAL> xDiff = (x - x_ref_);

    return P_ * xDiff;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::control_matrix_t
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::controlSecondDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    return control_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::control_state_matrix_t
TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateControlDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    return P_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::loadConfigFile(const std::string& filename,
    const std::string& termName,
    bool verbose)
{
    boost::property_tree::ptree pt;
    try
    {
        boost::property_tree::read_info(filename, pt);
    } catch (...)
    {
    }
    this->name_ = pt.get<std::string>(termName + ".name.", termName);

    loadMatrixCF(filename, "P", P_, termName);
    loadMatrixCF(filename, "x_des", x_ref_, termName);
    loadMatrixCF(filename, "u_des", u_ref_, termName);
    if (verbose)
    {
        std::cout << "Read P as P = \n" << P_ << std::endl;
        std::cout << "Read x_ref as x_ref = \n" << x_ref_.transpose() << std::endl;
        std::cout << "Read u_ref as u_ref = \n" << u_ref_.transpose() << std::endl;
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermMixed<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::updateReferenceState(
    const Eigen::Matrix<SCALAR_EVAL, STATE_DIM, 1>& newRefState)
{
    x_ref_ = newRefState;
}
}  // namespace optcon
}  // namespace ct
