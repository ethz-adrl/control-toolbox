/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

//! A linear state feedback controller
/*!
 * A general, time-varying linear state feedback controller with feedforward action of type.
 * It supports both continuous and discrete time.
 *
 * \f[
 *
 * u(x,t) = u_{ff}(t) + K(t) (x - x_{ref})
 *
 * \f]
 *
 * or
 *
 * \f[
 *
 * u(x,n) = u_{ff}[n] + K[n] (x - x_{ref})
 *
 * \f]
 *
 *
 * where \f$ u_{ff} \f$ is a time varying feedforward and \f$ K(t) \f$ a time-varying
 * linear feedback controller.
 *
 * @tparam STATE_DIM state vector size
 * @tparam CONTROL_DIM control vector size
 * @tparam SCALAR primitive type
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class StateFeedbackController : public Controller<STATE_DIM, CONTROL_DIM, SCALAR>,
                                public DiscreteController<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Controller<STATE_DIM, CONTROL_DIM, SCALAR> ContinuousBase;
    typedef DiscreteController<STATE_DIM, CONTROL_DIM, SCALAR> DiscreteBase;

    typedef typename DiscreteBase::state_vector_t state_vector_t;
    typedef typename DiscreteBase::control_vector_t control_vector_t;

    typedef StateVectorArray<STATE_DIM, SCALAR> state_vector_array_t;
    typedef ControlVectorArray<CONTROL_DIM, SCALAR> control_vector_array_t;
    typedef FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR> feedback_array_t;

    //! default constructor
    StateFeedbackController() : ContinuousBase(), DiscreteBase() {}
    //! copy constructor
    StateFeedbackController(const StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>& other);

    //! constructor
    /*!
     * Constructs a state feedback controller. The feedforward and feedback parts
     * get interpolated where required.
     *
     * Note: in the discrete setting, the initial time step is always assumed to be n=0
     *
     * @param uff feedforward controller.
     * @param K feedback controller.
     * @param deltaT discretization step
     * @param t0 initial time.
     * @param intType interpolation type
     */
    StateFeedbackController(const state_vector_array_t& x_ref,
        const control_vector_array_t& uff,
        const feedback_array_t& K,
        const SCALAR deltaT,
        const SCALAR t0 = 0.0,
        const InterpolationType& intType = ZOH);

    //! destructor
    virtual ~StateFeedbackController(){};

    //! deep cloning
    StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override;

    //! computes the control action in the continuous case
    /*!
     * evaluates the controller using interpolation where required using interpolation
     * @param state current state
     * @param t current time
     * @param controlAction resulting control action
     */
    virtual void computeControl(const state_vector_t& state, const SCALAR& t, control_vector_t& controlAction) override;

    //! computes the control action in the discrete case
    /*!
     * Evaluate the given controller for a given state and time index
     * returns the computed control action.
     *
     * @param state current state of the system
     * @param n current time index of the system
     * @param controlAction the corresponding control action
     */
    virtual void computeControl(const state_vector_t& state, const int n, control_vector_t& controlAction) override;

    //! updates the controller
    /*!
     * sets a new feedforward and feedback controller
     * @param uff feedforward control action
     * @param K feedback controller
     * @param times discretization times
     */
    void update(const DiscreteArray<state_vector_t>& x_ref,
        const DiscreteArray<control_vector_t>& uff,
        const DiscreteArray<FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>>& K,
        const tpl::TimeArray<SCALAR>& t);

    //! get reference state vector array (without timings)
    const DiscreteArray<state_vector_t>& x_ref() const;

    //! get feedforward array (without timings)
    const DiscreteArray<control_vector_t>& uff() const;

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
