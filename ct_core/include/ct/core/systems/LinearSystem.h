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
 * \tparam CONTROL_DIM size of input vector
 */
template <typename MANIFOLD, size_t CONTROL_DIM, bool CONT_T>
class LinearSystem : public ControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr size_t STATE_DIM = MANIFOLD::TangentDim;
    using SCALAR = typename MANIFOLD::Scalar;
    using Tangent = typename MANIFOLD::Tangent;

    using Base = ControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T>;
    using Time_t = typename Base::Time_t;

    using control_vector_t = ControlVector<CONTROL_DIM, SCALAR>;  //!< input vector type

    using state_matrix_t = StateMatrix<STATE_DIM, SCALAR>;                              //!< state Jacobian type
    using state_control_matrix_t = StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR>;  //!< input Jacobian type

    //! default constructor
    /*!
	 * @param type system type
	 */
    LinearSystem(const ct::core::SYSTEM_TYPE& type = ct::core::SYSTEM_TYPE::GENERAL) : Base(type) {}
    //! destructor
    virtual ~LinearSystem(){};

    //! deep cloning
    virtual LinearSystem<MANIFOLD, CONTROL_DIM, CONT_T>* clone() const override = 0;

    //! get the A matrix of a linear system
    /*!
	 * @param x state vector (required for linearizing non-linear systems, ignored for pure linear system)
	 * @param u input vector (required for linearizing non-linear systems, ignored for pure linear system)
	 * @param t current time
	 * @return A matrix
	 */
    virtual const state_matrix_t& getDerivativeState(const MANIFOLD& m,
        const control_vector_t& u,
        const Time_t t = Time_t(0.0)) = 0;

    //! get the B matrix of a linear system
    /*!
	 * @param x state vector (required for linearizing non-linear systems, ignored for pure linear system)
	 * @param u input vector (required for linearizing non-linear systems, ignored for pure linear system)
	 * @param t current time
	 * @return B matrix
	 */
    virtual const state_control_matrix_t& getDerivativeControl(const MANIFOLD& m,
        const control_vector_t& u,
        const Time_t t = Time_t(0.0)) = 0;

    //! compute the system dynamics
    /*!
	 * This computes the system dynamics. 
     * 
     * For the continuous-time case, we compute:
	 * \f[
	 *  \dot{x} = Ax + Bu
	 * \f]
     * 
     * For the discrete-time case, in order to be compliant with the manifold formulation, we compute
     * \f[
	 *  dx_{n+1} = (A-I)x_n + Bu_n
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
        dxdt = getDerivativeState(m, u, t) * this->lift(m) + getDerivativeControl(m, u, t) * u;
    }

    // discrete-time specialization
    template <typename T = void>
    typename std::enable_if<CONT_T == DISCRETE_TIME, T>::type computeControlledDynamics_specialized(const MANIFOLD& m,
        const Time_t& t,
        const control_vector_t& u,
        Tangent& dx)
    {
        dx = getDerivativeState(m, u, t) * this->lift(m) + getDerivativeControl(m, u, t) * u;
        dx = dx - this->lift(m);
    }

    /**
     * @brief Get both linear system matrices A and B in one call.
     * 
     * Motvation: For certain derived instances, it may be more efficient to compute A and B simulatenously. 
     * In that case, this method can be overloaded for maximum efficiency. As default case, it simply calls above
     * methods independently.
     * 
     * @param A matrix A of linear system
     * @param B matrix B of linear system
     * @param x the current state
     * @param u the current input
     * @param t the current time
     */
    virtual void getDerivatives(state_matrix_t& A,
        state_control_matrix_t& B,
        const MANIFOLD& m,
        const control_vector_t& u,
        const Time_t t = Time_t(0.0))
    {
        A = getDerivativeState(m, u, t);
        B = getDerivativeControl(m, u, t);
    }

    /**
     * @brief Get both linear system matrices A and B in one call, more verbose interface
     * 
     * @param A 
     * @param B 
     * @param m 
     * @param m_next 
     * @param u 
     * @param nSubsteps optional: number of substeps performed in the discretizer, e.g. for sensitivity integration
     * @param t 
     */
    virtual void getDerivatives(state_matrix_t& A,
        state_control_matrix_t& B,
        const MANIFOLD& m,
        const MANIFOLD& m_next,
        const control_vector_t& u,
        const size_t nSubsteps,
        const Time_t t = Time_t(0.0))
    {
        // we drop m_next in the default impl
        getDerivatives(A, B, m, u, t);
    }
};

}  // namespace core
}  // namespace ct
