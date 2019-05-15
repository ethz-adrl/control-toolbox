/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
class DiscreteSystem
{
public:
    typedef int time_t;                                           //!< the type of the time variable
    typedef StateVector<STATE_DIM, SCALAR> state_vector_t;        //<! state vector type
    typedef ControlVector<CONTROL_DIM, SCALAR> control_vector_t;  //<! control vector type

    //! constructor
    DiscreteSystem(const SYSTEM_TYPE& type = GENERAL) : type_(type) {}
    //! desctructor
    virtual ~DiscreteSystem() {}
    //! deep copy
    virtual DiscreteSystem* clone() const { throw std::runtime_error("DiscreteSystem: clone() not implemented"); };
    //! propagates the system dynamics forward by one step
    /*!
     * evaluates \f$ x_{n+1} = f(x_n, n) \f$ at a given state and index
     * @param state start state to propagate from
     * @param n time index to propagate the dynamics at
     * @param stateNext propagated state
     */
    virtual void propagateDynamics(const StateVector<STATE_DIM, SCALAR>& state,
        const time_t n,
        StateVector<STATE_DIM, SCALAR>& stateNext) = 0;

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
