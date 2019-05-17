/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "DiscreteSystem.h"
#include <ct/core/control/discrete_time/DiscreteController.h>

namespace ct {
namespace core {

//! A general, non-linear discrete dynamic system with a control input
/*!
 * This describes a general, non-linear discrete dynamic system of the following form
 *
 * \f[
 *  x_{n+1} = f(x_n,u_n,n)
 * \f]
 *
 * where \f$ x_{n} \f$ is the state, \f$ u_{n} \f$ the control input and \f$ n \f$ the time index.
 *
 * For implementing your own ControlledSystem, derive from this class.
 *
 * We generally assume that the Controller is a state and time index dependent function \f$ u_n = g(x_n,n) \f$
 * which allows any ControlledSystem to be re-written as a System of the form
 *
 * \f[
 *  x_{n+1} = f(x_n,u_n(x_n,n),n) = g(x_n,n)
 * \f]
 *
 * which can be forward propagated directly.
 *
 * @tparam STATE_DIM dimension of state vector
 * @tparam CONTROL_DIM dimension of input vector
 * @tparam SCALAR scalar type
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DiscreteControlledSystem : public DiscreteSystem<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename std::shared_ptr<DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> Ptr;

    typedef DiscreteSystem<STATE_DIM, CONTROL_DIM, SCALAR> Base;

    typedef typename Base::state_vector_t state_vector_t;
    typedef typename Base::control_vector_t control_vector_t;
    typedef typename Base::time_t time_t;

    //! default constructor
    /*!
	 * @param type system type
	 */
    DiscreteControlledSystem(const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL)
        : DiscreteSystem<STATE_DIM, CONTROL_DIM, SCALAR>(type), controller_(nullptr){};

    //! constructor
    /*!
	 *
	 * @param controller controller
	 * @param type system type
	 */
    DiscreteControlledSystem(std::shared_ptr<DiscreteController<STATE_DIM, CONTROL_DIM, SCALAR>> controller,
        const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL)
        : DiscreteSystem<STATE_DIM, CONTROL_DIM, SCALAR>(type), controller_(controller){};

    //! copy constructor
    DiscreteControlledSystem(const ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>& arg)
        : DiscreteSystem<STATE_DIM, CONTROL_DIM, SCALAR>(arg)
    {
        if (arg.controller_)
            controller_ = std::shared_ptr<DiscreteController<STATE_DIM, CONTROL_DIM, SCALAR>>(arg.controller_->clone());
    }

    //! destructor
    virtual ~DiscreteControlledSystem() = default;

    //! deep copy
    virtual DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override = 0;

    //! set a new controller
    /*!
	 * @param controller new controller
	 */
    void setController(const std::shared_ptr<DiscreteController<STATE_DIM, CONTROL_DIM, SCALAR>>& controller)
    {
        controller_ = controller;
    }

    //! get the controller instance
    /*!
	 * \todo remove this function (duplicate of getController() below)
	 * @param controller controller instance
	 */
    void getController(std::shared_ptr<DiscreteController<STATE_DIM, CONTROL_DIM, SCALAR>>& controller) const
    {
        controller = controller_;
    }

    //! get the controller instace
    /*!
	 * @return controller instance
	 */
    std::shared_ptr<DiscreteController<STATE_DIM, CONTROL_DIM, SCALAR>> getController() { return controller_; }
    //! propagates the system dynamics forward by one step
    /*!
	 * evaluates \f$ x_{n+1} = f(x_n, n) \f$ at a given state and index
	 * @param state start state to propagate from
	 * @param n time index to propagate the dynamics at
	 * @param stateNext the resulting propagated state
	 */
    virtual void propagateDynamics(const state_vector_t& state, const time_t n, state_vector_t& stateNext) override
    {
        control_vector_t controlAction;
        if (controller_)
            controller_->computeControl(state, n, controlAction);
        else
            controlAction.setZero();

        propagateControlledDynamics(state, n, controlAction, stateNext);
    }


    //! propagates the controlled system dynamics forward by one step
    /*!
	 * evaluates \f$ x_{n+1} = f(x_n, u_n, n) \f$ at a given state, control and index
	 * @param state start state to propagate from
	 * @param control the control input to apply. This is a constant control input applied to the continuous-time dynamics
	 * @param n time index to propagate the dynamics at
	 * @param stateNext the resulting propagated state
	 */
    virtual void propagateControlledDynamics(const state_vector_t& state,
        const time_t n,
        const control_vector_t& control,
        state_vector_t& stateNext) = 0;


protected:
    std::shared_ptr<DiscreteController<STATE_DIM, CONTROL_DIM, SCALAR>> controller_;  //!< the controller instance
};
}  // namespace core
}  // namespace ct
