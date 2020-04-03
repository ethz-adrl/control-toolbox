/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::TermQuadratic(const state_matrix_t& Q, const control_matrix_t& R)
    : Q_(Q), R_(R)
{
    x_ref_.setZero();  // default values
    u_ref_.setZero();
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::TermQuadratic()
{
    Q_.setConstant(9999);  // default values
    R_.setConstant(9999);
    x_ref_.setZero();
    u_ref_.setZero();
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::TermQuadratic(const state_matrix_t& Q,
    const control_matrix_t& R,
    const MANIFOLD& x_ref,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u_ref)
    : Q_(Q), R_(R), x_ref_(x_ref), u_ref_(u_ref)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::TermQuadratic(const std::string& configFile,
    const std::string& termName,
    bool verbose)
{
    loadConfigFile(configFile, termName, verbose);
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::TermQuadratic(
    const TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>& arg)
    : TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>(arg), Q_(arg.Q_), R_(arg.R_), x_ref_(arg.x_ref_), u_ref_(arg.u_ref_)
{
}


template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::~TermQuadratic()
{
}


template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>* TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::clone() const
{
    return new TermQuadratic(*this);
}


template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
void TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::setWeights(const state_matrix_t& Q, const control_matrix_t& R)
{
    Q_ = Q;
    R_ = R;
}


template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
const typename TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::state_matrix_t&
TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::getStateWeight() const
{
    return Q_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
typename TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::state_matrix_t&
TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::getStateWeight()
{
    return Q_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
const typename TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::control_matrix_t&
TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::getControlWeight() const
{
    return R_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
typename TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::control_matrix_t&
TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::getControlWeight()
{
    return R_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
void TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::setStateAndControlReference(const MANIFOLD& x_ref,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u_ref)
{
    x_ref_ = x_ref;
    u_ref_ = u_ref;
}


template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
auto TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::evaluate(const AD_MANIFOLD& x,
    const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
    const SCALAR& t) -> SCALAR
{
    Eigen::Matrix<SCALAR, STATE_DIM, 1> xDiff = (x - x_ref_.template cast<SCALAR>());
    Eigen::Matrix<SCALAR, CONTROL_DIM, 1> uDiff = (u - u_ref_.template cast<SCALAR>());

    return (xDiff.transpose() * Q_.template cast<SCALAR>() * xDiff +
            uDiff.transpose() * R_.template cast<SCALAR>() * uDiff)(0, 0);
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
auto TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::stateDerivative(const MANIFOLD& x,
    const ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> ct::core::StateVector<STATE_DIM, SCALAR_EVAL>
{
    typename MANIFOLD::Tangent xDiff = (x - x_ref_);

    return xDiff.transpose() * Q_.transpose() + xDiff.transpose() * Q_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
auto TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::stateSecondDerivative(const MANIFOLD& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> state_matrix_t
{
    return Q_ + Q_.transpose();
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
auto TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::controlDerivative(const MANIFOLD& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> core::ControlVector<CONTROL_DIM, SCALAR_EVAL>
{
    core::ControlVector<CONTROL_DIM, SCALAR_EVAL> uDiff = (u - u_ref_);

    return uDiff.transpose() * R_.transpose() + uDiff.transpose() * R_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
auto TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::controlSecondDerivative(const MANIFOLD& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> control_matrix_t
{
    return R_ + R_.transpose();
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
auto TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::stateControlDerivative(const MANIFOLD& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> control_state_matrix_t
{
    return control_state_matrix_t::Zero();
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
void TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::loadConfigFile(const std::string& filename,
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


template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
void TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::updateReferenceState(const MANIFOLD& x)
{
    x_ref_ = x;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
void TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::updateReferenceControl(
    const ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u)
{
    u_ref_ = u;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
MANIFOLD TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::getReferenceState() const
{
    return x_ref_;
}
}  // namespace optcon
}  // namespace ct
