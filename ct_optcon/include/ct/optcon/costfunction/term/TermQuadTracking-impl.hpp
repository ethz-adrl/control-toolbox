/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermQuadTracking<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermQuadTracking(const state_matrix_t& Q,
    const control_matrix_t& R,
    const core::InterpolationType& stateSplineType,
    const core::InterpolationType& controlSplineType,
    const bool trackControlTrajectory)
    : Q_(Q),
      R_(R),
      x_traj_ref_(stateSplineType),
      u_traj_ref_(controlSplineType),
      trackControlTrajectory_(trackControlTrajectory)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermQuadTracking<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermQuadTracking()
{
    // default values
    Q_.setIdentity();
    R_.setIdentity();
    trackControlTrajectory_ = false;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermQuadTracking<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermQuadTracking(
    const TermQuadTracking<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>& arg)
    : TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>(arg),
      Q_(arg.Q_),
      R_(arg.R_),
      x_traj_ref_(arg.x_traj_ref_),
      u_traj_ref_(arg.u_traj_ref_),
      trackControlTrajectory_(arg.trackControlTrajectory_)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermQuadTracking<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::~TermQuadTracking()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermQuadTracking<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>*
TermQuadTracking<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::clone() const
{
    return new TermQuadTracking(*this);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermQuadTracking<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::setWeights(const state_matrix_double_t& Q,
    const control_matrix_double_t& R)
{
    Q_ = Q.template cast<SCALAR_EVAL>();
    R_ = R.template cast<SCALAR_EVAL>();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermQuadTracking<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::setStateAndControlReference(
    const core::StateTrajectory<STATE_DIM>& xTraj,
    const core::ControlTrajectory<CONTROL_DIM>& uTraj)
{
    x_traj_ref_ = xTraj;
    u_traj_ref_ = uTraj;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
SCALAR TermQuadTracking<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::evaluate(
    const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
    const Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
    const SCALAR& t)
{
    return evalLocal<SCALAR>(x, u, t);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
ct::core::StateVector<STATE_DIM, SCALAR_EVAL>
TermQuadTracking<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateDerivative(
    const ct::core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    Eigen::Matrix<SCALAR_EVAL, STATE_DIM, 1> xDiff = x - x_traj_ref_.eval(t);

    return xDiff.transpose() * Q_.transpose() + xDiff.transpose() * Q_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermQuadTracking<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::state_matrix_t
TermQuadTracking<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateSecondDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    return Q_ + Q_.transpose();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
core::ControlVector<CONTROL_DIM, SCALAR_EVAL>
TermQuadTracking<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::controlDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, 1> uDiff;

    if (trackControlTrajectory_)
        uDiff = u - u_traj_ref_.eval(t);
    else
        uDiff = u;

    return uDiff.transpose() * R_.transpose() + uDiff.transpose() * R_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermQuadTracking<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::control_matrix_t
TermQuadTracking<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::controlSecondDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    return R_ + R_.transpose();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermQuadTracking<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::control_state_matrix_t
TermQuadTracking<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateControlDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    return control_state_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermQuadTracking<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::loadConfigFile(const std::string& filename,
    const std::string& termName,
    bool verbose)
{
    loadMatrixCF(filename, "Q", Q_, termName);
    loadMatrixCF(filename, "R", R_, termName);
    if (verbose)
    {
        std::cout << "Read Q as Q = \n" << Q_ << std::endl;
        std::cout << "Read R as R = \n" << R_ << std::endl;
    }
}
}  // namespace optcon
}  // namespace ct
