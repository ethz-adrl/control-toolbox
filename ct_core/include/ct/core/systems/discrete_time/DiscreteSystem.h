/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

template <typename MANIFOLD, typename SCALAR = typename MANIFOLD::Scalar>
class DiscreteSystem
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Tangent = typename MANIFOLD::Tangent;

    typedef int time_t;               //!< the type of the time variable
    typedef MANIFOLD state_vector_t;  //<! state vector type

    DiscreteSystem(const SYSTEM_TYPE& type = GENERAL) : type_(type) {}

    virtual ~DiscreteSystem() {}

    //! deep copy
    virtual DiscreteSystem* clone() const { throw std::runtime_error("DiscreteSystem: clone() not implemented"); };

    //! propagates the system dynamics forward by one step
    /*!
     * evaluates \f$ x_{n+1} = f(x_n, n) \f$ at a given state and index
     * @param state start state to propagate from
     * @param n time index to propagate the dynamics at
     * @param state_incr state increment in tangent space
     */
    virtual void propagateDynamics(const state_vector_t& state, const time_t n, Tangent& state_incr) = 0;

    //! get the type of system
    /*!
     * @return system type
     */
    SYSTEM_TYPE getType() const { return type_; }

protected:
    SYSTEM_TYPE type_;  //!< type of system
};
}  // namespace core
}  // namespace ct
