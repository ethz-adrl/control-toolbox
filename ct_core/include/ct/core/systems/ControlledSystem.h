/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/control/Controller.h>
#include <ct/core/systems/System.h>
#include <ct/core/types/arrays/TimeArray.h>


namespace ct {
namespace core {

//! A general, non-linear continuous-time dynamic system with a control input
/*!
 * This describes a general, non-linear dynamic system described by an Ordinary Differential Equation (ODE)
 * of the following form
 *
 * \f[
 *  \dot{x} = f(x,u,t)
 * \f]
 *
 * where \f$ x(t) \f$ is the state, \f$ u(t) \f$ the control input and \f$ t \f$ the time.
 *
 * For implementing your own ControlledSystem, derive from this class.
 *
 * We generally assume that the Controller is a state and time dependent function \f$ u = g(x,t) \f$
 * which allows any ControlledSystem to be re-written as a System of the form
 *
 * \f[
 *  \dot{x} = f(x(t),u(x,t),t) = g(x,t)
 * \f]
 *
 * which can be forward propagated in time with an Integrator.
 *
 * @tparam STATE_DIM dimension of state vector
 * @tparam CONTROL_DIM dimension of input vector
 * @tparam SCALAR scalar type
 */
template <typename MANIFOLD, size_t CONTROL_DIM, bool CONT_T>
class ControlledSystem : public System<MANIFOLD, CONT_T>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Base = System<MANIFOLD, CONT_T>;
    using SCALAR = typename MANIFOLD::Scalar;
    typedef typename Base::Time_t Time_t;

    using Tangent = typename MANIFOLD::Tangent;
    typedef typename std::shared_ptr<ControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T>> Ptr;

    using Controller_t = Controller<MANIFOLD, CONTROL_DIM, CONT_T>;
    using control_vector_t = typename Controller_t::control_vector_t;

    //! default constructor
    /*!
	 * @param type system type
	 */
    ControlledSystem(const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL);

    //! constructor
    /*!
	 *
	 * @param controller controller
	 * @param type system type
	 */
    ControlledSystem(std::shared_ptr<Controller_t> controller, const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL);

    ControlledSystem(const ControlledSystem& arg);

    virtual ~ControlledSystem();

    //! deep copy
    virtual ControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T>* clone() const override = 0;

    //! set a new controller
    /*!
	 * @param controller new controller
	 */
    void setController(const std::shared_ptr<Controller_t>& controller);

    //! get the controller instance
    /*!
	 * \todo remove this function (duplicate of getController() below)
	 * @param controller controller instance
	 */
    void getController(std::shared_ptr<Controller_t>& controller) const;

    //! get the controller instace
    /*!
	 * @return controller instance
	 */
    std::shared_ptr<Controller_t> getController();

    virtual void computeDynamics(const MANIFOLD& state, const Time_t& t, Tangent& derivative) override;

    /*!
     * @brief compute the dynamics of the system
	 * Compute the state derivative by evaluating the system dynamics for a given state. This
	 * calls the controller first and then calls computeControlledDynamics() with the current state
	 * and the resulting control signal.
	 *
	 * \note Generally, this function does not need to be overloaded. Better overload computeControlledDynamics().
	 *
	 * @param state current state
	 * @param t current time
	 * @param derivative state derivative
	 */
    virtual void computeControlledDynamics(const MANIFOLD& state,
        const Time_t& tn,
        const control_vector_t& control,
        Tangent& derivative) = 0;


    control_vector_t getLastControlAction() const;

protected:
    std::shared_ptr<Controller_t> controller_;  //!< the controller instance

    control_vector_t controlAction_;
};
}  // namespace core
}  // namespace ct
