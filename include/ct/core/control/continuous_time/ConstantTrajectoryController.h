/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "Controller.h"

namespace ct {
namespace core {

//! A constant controller
/*!
 * Implements a controller that is time and state invariant, i.e. fully constant.
 * This class is useful to integrate a ControlledSystem forward subject to a
 * constant control input.
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ConstantTrajectoryController : public Controller<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Default constructor
    /*!
	 * Sets the control signal to zero
	 */
    ConstantTrajectoryController() {}
    //! Constructor
    /*!
	 * Initializes the control to a fixed value
	 * @param u The fixed control signal
	 */
    ConstantTrajectoryController(const ControlVectorArray<CONTROL_DIM, SCALAR>& u,
        const StateVectorArray<STATE_DIM, SCALAR>& x)
        : uff_(u), x_(x)
    {
    }

    //! Copy constructor
    ConstantTrajectoryController(const ConstantTrajectoryController<STATE_DIM, CONTROL_DIM, SCALAR>& other)
        : Controller<STATE_DIM, CONTROL_DIM, SCALAR>(other), x_(other.x_), uff_(other.uff_)
    {
    }

    //! Destructor
    ~ConstantTrajectoryController() {}
    //! Clone operator
    /*!
	 * Clones the controller. Used for cloning ControlledSystem's
	 * @return pointer to cloned controller
	 */
    ConstantTrajectoryController<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
    {
        return new ConstantTrajectoryController<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
    }

    //! Computes current control
    /*!
	 * Returns the fixed control signal. Therefore, the return value is invariant
	 * to the parameters.
	 * @param state The current state of the system (ignored)
	 * @param t The time of the system (ignored)
	 * @param controlAction The (fixed) control action
	 */
    void computeControl(const StateVector<STATE_DIM, SCALAR>& state,
        const SCALAR& t,
        ControlVector<CONTROL_DIM, SCALAR>& controlAction) override
    {
        throw std::runtime_error("not implemented");
    }

    //! Sets the control signal
    /*!
	 *
	 * @param u The fixed control signal
	 */
    void setControlVectorArray(const ControlVectorArray<CONTROL_DIM, SCALAR>& u) { uff_ = u; }
    //! Get the fixed control signal
    /*!
	 */
    const ControlVectorArray<CONTROL_DIM, SCALAR>& getControlVectorArray() const { return uff_; }
    //! Sets the state trajectory
    /*!
	 *
	 * @param u The fixed state trajectory
	 */
    void setStateVectorArray(const StateVectorArray<STATE_DIM, SCALAR>& x) { x_ = x; }
    //! Get the fixed state trajectory
    /*!
	 *
	 *
	 */
    const StateVectorArray<STATE_DIM, SCALAR>& getStateVectorArray() const { return x_; }
private:
    ControlVectorArray<CONTROL_DIM, SCALAR> uff_;  //! feedforward control trajectory
    StateVectorArray<STATE_DIM, SCALAR> x_;
};
}
}
