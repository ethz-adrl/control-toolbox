/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermQuadratic(const state_matrix_t& Q,
    const control_matrix_t& R)
    : Q_(Q), R_(R)
{
    x_ref_.setZero();  // default values
    u_ref_.setZero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermQuadratic()
{
    Q_.setConstant(9999);  // default values
    R_.setConstant(9999);
    x_ref_.setZero();
    u_ref_.setZero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermQuadratic(const state_matrix_t& Q,
    const control_matrix_t& R,
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x_ref,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u_ref)
    : Q_(Q), R_(R), x_ref_(x_ref), u_ref_(u_ref)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermQuadratic(const std::string& configFile,
    const std::string& termName,
    bool verbose)
{
    loadConfigFile(configFile, termName, verbose);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermQuadratic(
    const TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>& arg)
    : TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>(arg),
      Q_(arg.Q_),
      R_(arg.R_),
      x_ref_(arg.x_ref_),
      u_ref_(arg.u_ref_)
{
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::~TermQuadratic()
{
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>*
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::clone() const
{
    return new TermQuadratic(*this);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::setWeights(
    const Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM>& Q,
    const Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM>& R)
{
    Q_ = Q;
    R_ = R;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
const typename TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::state_matrix_t&
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::getStateWeight() const
{
    return Q_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::state_matrix_t&
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::getStateWeight()
{
    return Q_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
const typename TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::control_matrix_t&
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::getControlWeight() const
{
    return R_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::control_matrix_t&
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::getControlWeight()
{
    return R_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::setStateAndControlReference(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x_ref,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u_ref)
{
    x_ref_ = x_ref;
    u_ref_ = u_ref;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
SCALAR TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::evaluate(
    const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
    const Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
    const SCALAR& t)
{
    return evalLocal<SCALAR>(x, u, t);
}

#ifdef CPPADCG
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
ct::core::ADCGScalar TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::evaluateCppadCg(
    const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
    const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
    ct::core::ADCGScalar t)
{
    return evalLocal<ct::core::ADCGScalar>(x, u, t);
}
#endif

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
ct::core::StateVector<STATE_DIM, SCALAR_EVAL>
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateDerivative(
    const ct::core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    ct::core::StateVector<STATE_DIM, SCALAR_EVAL> xDiff = (x - x_ref_);

    return xDiff.transpose() * Q_.transpose() + xDiff.transpose() * Q_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::state_matrix_t
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateSecondDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    return Q_ + Q_.transpose();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
core::ControlVector<CONTROL_DIM, SCALAR_EVAL>
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::controlDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    core::ControlVector<CONTROL_DIM, SCALAR_EVAL> uDiff = (u - u_ref_);

    return uDiff.transpose() * R_.transpose() + uDiff.transpose() * R_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::control_matrix_t
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::controlSecondDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    return R_ + R_.transpose();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::control_state_matrix_t
TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateControlDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    return control_state_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::loadConfigFile(const std::string& filename,
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

    loadMatrixCF(filename, "Q", Q_, termName);
    loadMatrixCF(filename, "R", R_, termName);
    loadMatrixCF(filename, "x_des", x_ref_, termName);
    loadMatrixCF(filename, "u_des", u_ref_, termName);
    if (verbose)
    {
        std::cout << "Read Q as Q = \n" << Q_ << std::endl;
        std::cout << "Read R as R = \n" << R_ << std::endl;
        std::cout << "Read x_des as x_des = \n" << x_ref_.transpose() << std::endl;
        std::cout << "Read u_des as u_des = \n" << u_ref_.transpose() << std::endl;
    }
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::updateReferenceState(
    const Eigen::Matrix<SCALAR_EVAL, STATE_DIM, 1>& newRefState)
{
    x_ref_ = newRefState;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::updateReferenceControl(
    const Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, 1>& newRefControl)
{
    u_ref_ = newRefControl;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
Eigen::Matrix<SCALAR_EVAL, STATE_DIM, 1> TermQuadratic<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::getReferenceState()
    const
{
    return x_ref_;
}
}  // namespace optcon
}  // namespace ct
