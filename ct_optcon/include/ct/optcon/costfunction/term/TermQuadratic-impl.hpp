/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <typename MANIFOLD, size_t CONTROL_DIM>
TermQuadratic<MANIFOLD, CONTROL_DIM>::TermQuadratic(const state_matrix_t& Q, const control_matrix_t& R) : Q_(Q), R_(R)
{
    x_ref_.setZero();  // default values
    u_ref_.setZero();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
TermQuadratic<MANIFOLD, CONTROL_DIM>::TermQuadratic()
{
    Q_.setConstant(9999);  // default values
    R_.setConstant(9999);
    x_ref_.setZero();
    u_ref_.setZero();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
TermQuadratic<MANIFOLD, CONTROL_DIM>::TermQuadratic(const state_matrix_t& Q,
    const control_matrix_t& R,
    const MANIFOLD& x_ref,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u_ref)
    : Q_(Q), R_(R), x_ref_(x_ref), u_ref_(u_ref)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
TermQuadratic<MANIFOLD, CONTROL_DIM>::TermQuadratic(const std::string& configFile,
    const std::string& termName,
    bool verbose)
{
    loadConfigFile(configFile, termName, verbose);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
TermQuadratic<MANIFOLD, CONTROL_DIM>::TermQuadratic(const TermQuadratic<MANIFOLD, CONTROL_DIM>& arg)
    : TermBase<MANIFOLD, CONTROL_DIM>(arg), Q_(arg.Q_), R_(arg.R_), x_ref_(arg.x_ref_), u_ref_(arg.u_ref_)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
TermQuadratic<MANIFOLD, CONTROL_DIM>::~TermQuadratic()
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
TermQuadratic<MANIFOLD, CONTROL_DIM>* TermQuadratic<MANIFOLD, CONTROL_DIM>::clone() const
{
    return new TermQuadratic(*this);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void TermQuadratic<MANIFOLD, CONTROL_DIM>::setWeights(const state_matrix_t& Q, const control_matrix_t& R)
{
    Q_ = Q;
    R_ = R;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
const typename TermQuadratic<MANIFOLD, CONTROL_DIM>::state_matrix_t&
TermQuadratic<MANIFOLD, CONTROL_DIM>::getStateWeight() const
{
    return Q_;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename TermQuadratic<MANIFOLD, CONTROL_DIM>::state_matrix_t& TermQuadratic<MANIFOLD, CONTROL_DIM>::getStateWeight()
{
    return Q_;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
const typename TermQuadratic<MANIFOLD, CONTROL_DIM>::control_matrix_t&
TermQuadratic<MANIFOLD, CONTROL_DIM>::getControlWeight() const
{
    return R_;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename TermQuadratic<MANIFOLD, CONTROL_DIM>::control_matrix_t&
TermQuadratic<MANIFOLD, CONTROL_DIM>::getControlWeight()
{
    return R_;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void TermQuadratic<MANIFOLD, CONTROL_DIM>::setStateAndControlReference(const EVAL_MANIFOLD& x_ref,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u_ref)
{
    x_ref_ = x_ref;
    u_ref_ = u_ref;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto TermQuadratic<MANIFOLD, CONTROL_DIM>::evaluate(const MANIFOLD& x,
    const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
    const SCALAR& t) -> SCALAR
{
    Eigen::Matrix<SCALAR, STATE_DIM, 1> xDiff = (x - x_ref_.template cast<SCALAR>());
    Eigen::Matrix<SCALAR, CONTROL_DIM, 1> uDiff = (u - u_ref_.template cast<SCALAR>());

    return (xDiff.transpose() * Q_.template cast<SCALAR>() * xDiff +
            uDiff.transpose() * R_.template cast<SCALAR>() * uDiff)(0, 0);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto TermQuadratic<MANIFOLD, CONTROL_DIM>::stateDerivative(const EVAL_MANIFOLD& x,
    const ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> ct::core::StateVector<STATE_DIM, SCALAR_EVAL>
{
    typename EVAL_MANIFOLD::Tangent xDiff = (x - x_ref_);

    return xDiff.transpose() * Q_.transpose() + xDiff.transpose() * Q_;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto TermQuadratic<MANIFOLD, CONTROL_DIM>::stateSecondDerivative(const EVAL_MANIFOLD& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> state_matrix_t
{
    return Q_ + Q_.transpose();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto TermQuadratic<MANIFOLD, CONTROL_DIM>::controlDerivative(const EVAL_MANIFOLD& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> core::ControlVector<CONTROL_DIM, SCALAR_EVAL>
{
    core::ControlVector<CONTROL_DIM, SCALAR_EVAL> uDiff = (u - u_ref_);

    return uDiff.transpose() * R_.transpose() + uDiff.transpose() * R_;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto TermQuadratic<MANIFOLD, CONTROL_DIM>::controlSecondDerivative(const EVAL_MANIFOLD& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> control_matrix_t
{
    return R_ + R_.transpose();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto TermQuadratic<MANIFOLD, CONTROL_DIM>::stateControlDerivative(const EVAL_MANIFOLD& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> control_state_matrix_t
{
    return control_state_matrix_t::Zero();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void TermQuadratic<MANIFOLD, CONTROL_DIM>::loadConfigFile(const std::string& filename,
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

template <typename MANIFOLD, size_t CONTROL_DIM>
void TermQuadratic<MANIFOLD, CONTROL_DIM>::updateReferenceState(const EVAL_MANIFOLD& x)
{
    x_ref_ = x;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void TermQuadratic<MANIFOLD, CONTROL_DIM>::updateReferenceControl(
    const ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u)
{
    u_ref_ = u;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto TermQuadratic<MANIFOLD, CONTROL_DIM>::getReferenceState() const -> EVAL_MANIFOLD
{
    return x_ref_;
}
}  // namespace optcon
}  // namespace ct
