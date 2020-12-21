/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/types/EuclideanState.h>
#include <ct/core/types/ControlVector.h>
#include <ct/core/types/StateMatrix.h>
#include <ct/core/types/StateControlMatrix.h>

#include <ct/core/systems/ControlledSystem.h>

namespace ct {
namespace core {

//! interface class for a general linear system or linearized system
/*!
 * Defines the interface for a linear system
 *
 * \tparam STATE_DIM size of state vector
 */
template <typename MANIFOLD, bool CONT_T>
class LinearSystem : public ControlledSystem<MANIFOLD, CONT_T>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr size_t STATE_DIM = MANIFOLD::TangentDim;
    using SCALAR = typename MANIFOLD::Scalar;
    using Tangent = typename MANIFOLD::Tangent;

    using Base = ControlledSystem<MANIFOLD, CONT_T>;
    using Time_t = typename Base::Time_t;

    using control_vector_t = ControlVector<SCALAR>;                        //!< input vector type
    using state_matrix_t = StateMatrix<STATE_DIM, SCALAR>;                 //!< state Jacobian type
    using state_control_matrix_t = StateControlMatrix<STATE_DIM, SCALAR>;  //!< input Jacobian type

    LinearSystem() = delete;

    LinearSystem(const int control_dim, const ct::core::SYSTEM_TYPE& type = ct::core::SYSTEM_TYPE::GENERAL)
        : Base(control_dim, type)
    {
    }

    LinearSystem(const MANIFOLD& m_ref,
        const control_vector_t& u_ref,
        const ct::core::SYSTEM_TYPE& type = ct::core::SYSTEM_TYPE::GENERAL)
        : Base(u_ref.rows(), type), m_ref_(m_ref), u_ref_(u_ref)
    {
    }

    //! deep cloning
    virtual LinearSystem<MANIFOLD, CONT_T>* clone() const override = 0;

    void SetLinearizationPoint(const MANIFOLD& m_ref, const control_vector_t& u_ref)
    {
        m_ref_ = m_ref;
        u_ref_ = u_ref;
    }

    //! get the A matrix of a linear system around m_ref, u_ref
    /*!
	 * @param t current time
	 * @return A matrix
	 */
    virtual const state_matrix_t& getDerivativeState(const Time_t t = Time_t(0.0)) = 0;

    //! get the B matrix of a linear system around m_ref, u_ref
    /*!
	 * @param x state vector (required for linearizing non-linear systems, ignored for pure linear system)
	 * @param u input vector (required for linearizing non-linear systems, ignored for pure linear system)
	 * @param t current time
	 * @return B matrix
	 */
    virtual const state_control_matrix_t& getDerivativeControl(const Time_t t = Time_t(0.0)) = 0;

    //! compute the system dynamics
    /*!
	 * This computes the system dynamics. 
     * 
     * For the continuous-time case, we compute:
	 * \f[
	 *  \dot{x} = A (m-m_ref) + B(u-u_ref)  // TODO, the adjoint are missing here
	 * \f]
     * 
     * For the discrete-time case, in order to be compliant with the manifold formulation, we compute
     * \f[
	 *  dx_{n+1} = (A-I)(x_n-x_ref) + B(u_n-u_ref)
	 * \f]
     * Therefore, 
     * \f[
     *  x_{n+1} = x_n + dx_{n+1}
     * \f]
	 * @param state current state
	 * @param t current time
	 * @param control control input
	 * @param derivative state derivative
	 */
    virtual void computeControlledDynamics(const MANIFOLD& m,
        const Time_t& t,
        const control_vector_t& u,
        Tangent& dxdt) override
    {
        computeControlledDynamics_specialized(m, t, u, dxdt);
    }

    // continuous-time specialization
    template <typename T = void>
    typename std::enable_if<CONT_T == CONTINUOUS_TIME, T>::type computeControlledDynamics_specialized(const MANIFOLD& m,
        const Time_t& t,
        const control_vector_t& u,
        Tangent& dxdt)
    {
        // dx expressed in tangent space of m_ref_
        Eigen::Matrix<SCALAR, MANIFOLD::TangentDim, MANIFOLD::TangentDim> Jl, Jr; 
        auto dx = m.rminus(m_ref_, Jl, Jr);

// TODO: there are some things equivalent. -- Need to understand the relaton between Jr, Jl and Adj.
std::cout << "Jl" << std::endl << Jl << std::endl;
std::cout << "m.between(m_ref_).adj()" << std::endl << m.between(m_ref_).adj() << std::endl;

        auto m_ref_adj_m = Jl; // m.between(m_ref_).adj();
        // control input needs to be transported to m_ref_ to be compliant with u_ref
        auto u_in_m_ref = m_ref_adj_m * u;
        auto dxdt_in_m_ref = getDerivativeState(t) * dx + getDerivativeControl(t) * (u_in_m_ref - u_ref_);
        // TODO: dxdt needs to be transported back to m.
        dxdt = /*dx.adj().inverse()*/ /*m_ref_adj_m.inverse() */ Jl.inverse() * dxdt_in_m_ref;
    }

    // discrete-time specialization
    template <typename T = void>
    typename std::enable_if<CONT_T == DISCRETE_TIME, T>::type computeControlledDynamics_specialized(const MANIFOLD& m,
        const Time_t& t,
        const control_vector_t& u,
        Tangent& dx)
    {
        throw std::runtime_error(" not impl yet.");
        // dx = getDerivativeState(m, u, t) * this->lift(m) + getDerivativeControl(m, u, t) * u;
        // dx = dx - this->lift(m);
    }

    virtual void getDerivatives(state_matrix_t& A, state_control_matrix_t& B, const Time_t t = Time_t(0.0))
    {
        A = getDerivativeState(t);
        B = getDerivativeControl(t);
    }

    virtual void getDerivatives(state_matrix_t& A,
        state_control_matrix_t& B,
        const size_t nSubsteps,
        const Time_t t = Time_t(0.0))
    {
        // we drop the substeps in the default impl
        getDerivatives(A, B, t);
    }


protected:
    // Manifold state reference set point for linearization.
    MANIFOLD m_ref_;
    // reference control for linearization
    control_vector_t u_ref_;
};

}  // namespace core
};  // namespace ct
