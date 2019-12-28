/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermSmoothAbs<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermSmoothAbs(
    const core::StateVector<STATE_DIM, SCALAR_EVAL> a,
    const core::StateVector<STATE_DIM, SCALAR_EVAL> x_ref,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL> b,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL> u_ref,
    const SCALAR_EVAL alpha)
    : a_(a), x_ref_(x_ref), b_(b), u_ref_(u_ref), alphaSquared_(alpha * alpha)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermSmoothAbs<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermSmoothAbs(const TermSmoothAbs& arg)
    : TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>(arg),
      a_(arg.a_),
      x_ref_(arg.x_ref_),
      b_(arg.b_),
      u_ref_(arg.u_ref_),
      alphaSquared_(arg.alphaSquared_)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermSmoothAbs<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>*
TermSmoothAbs<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::clone() const
{
    return new TermSmoothAbs<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>(*this);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermSmoothAbs<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::~TermSmoothAbs()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
SCALAR TermSmoothAbs<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::evaluate(
    const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
    const Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
    const SCALAR& t)
{
    return evalLocal<SCALAR>(x, u, t);
}

#ifdef CPPADCG
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
ct::core::ADCGScalar TermSmoothAbs<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::evaluateCppadCg(
    const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
    const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
    ct::core::ADCGScalar t)
{
    return evalLocal<ct::core::ADCGScalar>(x, u, t);
}
#endif

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
core::StateVector<STATE_DIM, SCALAR_EVAL> TermSmoothAbs<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    return (a_.array() * (x - x_ref_).array() *
            ((x - x_ref_).array().square() + Eigen::Array<SCALAR_EVAL, STATE_DIM, 1>::Ones() * alphaSquared_)
                .sqrt()
                .inverse())
        .matrix();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermSmoothAbs<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::state_matrix_t
TermSmoothAbs<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateSecondDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    // return as diagonal matrix
    return (a_.array() * alphaSquared_ *
            (Eigen::Array<SCALAR_EVAL, STATE_DIM, 1>::Ones() * alphaSquared_ + (x - x_ref_).array().square())
                .pow(-3.0 / 2.0))
        .matrix()
        .asDiagonal();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
core::ControlVector<CONTROL_DIM, SCALAR_EVAL>
TermSmoothAbs<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::controlDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    return (b_.array() * (u - u_ref_).array() *
            ((u - u_ref_).array().square() + Eigen::Array<SCALAR_EVAL, CONTROL_DIM, 1>::Ones() * alphaSquared_)
                .sqrt()
                .inverse())
        .matrix();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermSmoothAbs<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::control_matrix_t
TermSmoothAbs<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::controlSecondDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    // return as diagonal matrix
    return (b_.array() * alphaSquared_ *
            (Eigen::Array<SCALAR_EVAL, CONTROL_DIM, 1>::Ones() * alphaSquared_ + (u - u_ref_).array().square())
                .pow(-3.0 / 2.0))
        .matrix()
        .asDiagonal();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermSmoothAbs<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::control_state_matrix_t
TermSmoothAbs<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateControlDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    return control_state_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermSmoothAbs<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::loadConfigFile(const std::string& filename,
    const std::string& termName,
    bool verbose)
{
    // read in the file and put the valus in a_ and b_
    a_.setZero();
    x_ref_.setZero();
    b_.setZero();
    u_ref_.setZero();

    loadMatrixCF(filename, "a", a_, termName);
    loadMatrixCF(filename, "x_ref", x_ref_, termName);
    loadMatrixCF(filename, "b", b_, termName);
    loadMatrixCF(filename, "u_ref", u_ref_, termName);
    loadScalarCF(filename, "alpha", alphaSquared_, termName);
    alphaSquared_ *= alphaSquared_;

    if (verbose)
    {
        std::cout << "Reading " << termName;
        std::cout << "\nRead a as a= " << a_.transpose();
        std::cout << "\nRead x_ref as x_ref= " << x_ref_.transpose();
        std::cout << "\nRead b as b= " << b_.transpose();
        std::cout << "\nRead u_ref as u_ref= " << u_ref_.transpose();
        std::cout << "\nRead alpha^2 as " << alphaSquared_ << std::endl;
    }
}
}  // namespace optcon
}  // namespace ct
