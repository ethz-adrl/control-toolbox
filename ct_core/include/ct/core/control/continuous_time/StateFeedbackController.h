/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <iostream>

#include "Controller.h"

#include <ct/core/types/arrays/MatrixArrays.h>
#include <ct/core/types/trajectories/MatrixTrajectories.h>

namespace ct {
namespace core {

//! A linear state feedback controller
/*!
 * A general, discrete, time-varying linear state feedback controller with feedforward action of type
 *
 * \f[
 *
 * u(x,t) = u_{ff}(t) + K(t) (x - x_{ref})
 *
 * \f]
 *
 * where \f$ u_{ff} \f$ is a time varying feedforward and \f$ K(t) \f$ a time-varying
 * linear feedback controller.
 *
 * @tparam STATE_DIM state vector size
 * @tparam CONTROL_DIM control vector size
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class StateFeedbackController : public Controller<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! The base (interface) class
    typedef Controller<STATE_DIM, CONTROL_DIM, SCALAR> ControllerBase;

    //! default constructor
    StateFeedbackController();

    //! constructor
    /*!
	 * Constructs a state feedback controller. The feedforward and feedback parts
	 * get interpolated where required.
	 * @param uff feedforward controller.
	 * @param K feedback controller.
	 * @param deltaT discretization step
	 * @param t0 initial time
	 * @param intType interpolation type
	 */
    StateFeedbackController(const StateVectorArray<STATE_DIM, SCALAR>& x_ref,
        const ControlVectorArray<CONTROL_DIM, SCALAR>& uff,
        const FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR>& K,
        const SCALAR& deltaT,
        const SCALAR& t0 = 0.0,
        const InterpolationType& intType = ZOH);

    //! copy constructor
    StateFeedbackController(const StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>& other);

    //! destructor
    virtual ~StateFeedbackController();

    //! deep cloning, required by ControlledSystem
    StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override;

    //! computes the control action
    /*!
	 * evaluates the controller using interpolation where required using Interpolation
	 * @param state current state
	 * @param t current time
	 * @param controlAction resulting control action
	 */
    virtual void computeControl(const StateVector<STATE_DIM, SCALAR>& state,
        const SCALAR& t,
        ControlVector<CONTROL_DIM, SCALAR>& controlAction) override;

    //! updates the controller
    /*!
	 * sets a new feedforward and feedback controller
	 * @param uff feedforward control action
	 * @param K feedback controller
	 * @param times discretization times
	 */
    void update(const DiscreteArray<StateVector<STATE_DIM, SCALAR>>& x_ref,
        const DiscreteArray<ControlVector<CONTROL_DIM, SCALAR>>& uff,
        const DiscreteArray<FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>>& K,
        const tpl::TimeArray<SCALAR>& t);

    //! get reference state vector array (without timings)
    const DiscreteArray<StateVector<STATE_DIM, SCALAR>>& x_ref() const;

    //! get feedforward array (without timings)
    const DiscreteArray<ControlVector<CONTROL_DIM, SCALAR>>& uff() const;

    //! get feedback array (without timings)
    const DiscreteArray<FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>>& K() const;

    //! get time array
    const tpl::TimeArray<SCALAR>& time() const;

    //! get a reference to the feedforward trajectory
    StateTrajectory<STATE_DIM, SCALAR>& getReferenceStateTrajectory();

    //! get a reference to the feedforward trajectory
    const StateTrajectory<STATE_DIM, SCALAR>& getReferenceStateTrajectory() const;

    //! get a reference to the feedforward trajectory
    ControlTrajectory<CONTROL_DIM, SCALAR>& getFeedforwardTrajectory();

    //! get a reference to the feedforward trajectory
    const ControlTrajectory<CONTROL_DIM, SCALAR>& getFeedforwardTrajectory() const;

    //! get a reference to the feedback trajectory
    FeedbackTrajectory<STATE_DIM, CONTROL_DIM, SCALAR>& getFeedbackTrajectory();

    //! get a reference to the feedback trajectory
    const FeedbackTrajectory<STATE_DIM, CONTROL_DIM, SCALAR>& getFeedbackTrajectory() const;

    //!  extracts a physically meaningful control trajectory from the given state-feedback law and a reference state trajectory
    void extractControlTrajectory(const StateTrajectory<STATE_DIM, SCALAR>& x_traj,
        ControlTrajectory<CONTROL_DIM, SCALAR>& u_traj);

private:
    StateTrajectory<STATE_DIM, SCALAR> x_ref_;              //! state reference trajectory
    ControlTrajectory<CONTROL_DIM, SCALAR> uff_;            //! feedforward control trajectory
    FeedbackTrajectory<STATE_DIM, CONTROL_DIM, SCALAR> K_;  //! feedback control trajectory
};

}  // core
}  // ct
