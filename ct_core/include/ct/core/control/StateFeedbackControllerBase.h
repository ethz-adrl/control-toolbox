/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

//! Base class for discrete and continuous state feedback controller
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class StateFeedbackControllerBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! default constructor
    StateFeedbackControllerBase() {}

    //! copy constructor
    StateFeedbackControllerBase(const StateFeedbackControllerBase<STATE_DIM, CONTROL_DIM, SCALAR>& other);

    StateFeedbackControllerBase(const StateVectorArray<STATE_DIM, SCALAR>& x_ref,
        const ControlVectorArray<CONTROL_DIM, SCALAR>& uff,
        const FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR>& K,
        const SCALAR deltaT,
        const SCALAR t0 = 0.0,
        const InterpolationType& intType = ZOH);

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

protected:
    StateTrajectory<STATE_DIM, SCALAR> x_ref_;              //! state reference trajectory
    ControlTrajectory<CONTROL_DIM, SCALAR> uff_;            //! feedforward control trajectory
    FeedbackTrajectory<STATE_DIM, CONTROL_DIM, SCALAR> K_;  //! feedback control trajectory
};

}  // namespace core
}  // namespace ct
