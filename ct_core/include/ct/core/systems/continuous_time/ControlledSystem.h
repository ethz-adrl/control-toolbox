/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/control/continuous_time/Controller.h>
#include <ct/core/systems/continuous_time/System.h>
#include <ct/core/types/arrays/TimeArray.h>


namespace ct {
namespace core {

//! A general, non-linear dynamic system with a control input
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
template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR = typename MANIFOLD::Scalar>
class ControlledSystem : public System<MANIFOLD, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef System<MANIFOLD, SCALAR> Base;
    typedef typename std::shared_ptr<ControlledSystem<MANIFOLD, CONTROL_DIM, SCALAR>> Ptr;
    typedef typename Base::Time_t Time_t;
    using Tangent = typename MANIFOLD::Tangent;

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
    ControlledSystem(std::shared_ptr<ct::core::Controller<MANIFOLD, CONTROL_DIM, SCALAR>> controller,
        const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL);

    ControlledSystem(const ControlledSystem& arg);

    virtual ~ControlledSystem();

    //! deep copy
    virtual ControlledSystem<MANIFOLD, CONTROL_DIM, SCALAR>* clone() const override = 0;

    //! set a new controller
    /*!
	 * @param controller new controller
	 */
    void setController(const std::shared_ptr<Controller<MANIFOLD, CONTROL_DIM, SCALAR>>& controller);

    //! get the controller instance
    /*!
	 * \todo remove this function (duplicate of getController() below)
	 * @param controller controller instance
	 */
    void getController(std::shared_ptr<Controller<MANIFOLD, CONTROL_DIM, SCALAR>>& controller) const;

    //! get the controller instace
    /*!
	 * @return controller instance
	 */
    std::shared_ptr<Controller<MANIFOLD, CONTROL_DIM, SCALAR>> getController();

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
        const Time_t& t,
        const ControlVector<CONTROL_DIM, SCALAR>& control,
        Tangent& derivative) = 0;


    ControlVector<CONTROL_DIM, SCALAR> getLastControlAction();

protected:
    std::shared_ptr<Controller<MANIFOLD, CONTROL_DIM, SCALAR>> controller_;  //!< the controller instance

    ControlVector<CONTROL_DIM, SCALAR> controlAction_;
};
}  // namespace core
}  // namespace ct
