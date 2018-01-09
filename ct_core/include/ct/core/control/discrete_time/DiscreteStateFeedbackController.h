/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

//! A discrete linear state feedback controller
/*!
 * A general, discrete, time-varying linear state feedback controller with feedforward action of type
 *
 * \f[
 *
 * u(x,n) = u_{ff}[n] + K[n] (x - x_{ref})
 *
 * \f]
 *
 * where \f$ u_{ff} \f$ is a time varying feedforward and \f$ K[n] \f$ a time-varying
 * linear feedback controller.
 *
 * @tparam STATE_DIM state vector size
 * @tparam CONTROL_DIM control vector size
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DiscreteStateFeedbackController : public DiscreteController<STATE_DIM, CONTROL_DIM, SCALAR>,
                                        public StateFeedbackControllerBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef DiscreteController<STATE_DIM, CONTROL_DIM, SCALAR> ControllerBase;

    typedef StateFeedbackControllerBase<STATE_DIM, CONTROL_DIM, SCALAR> StateFeedbackBase;

    typedef typename ControllerBase::state_vector_t state_vector_t;
    typedef typename ControllerBase::control_vector_t control_vector_t;

    typedef StateVectorArray<STATE_DIM, SCALAR> state_vector_array_t;
    typedef ControlVectorArray<CONTROL_DIM, SCALAR> control_vector_array_t;
    typedef FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR> feedback_array_t;

    //! default constructor
    DiscreteStateFeedbackController() {}

    //! constructor
    /*!
     * Constructs a state feedback controller. The feedforward and feedback parts
     * get interpolated where required.
     * @param uff feedforward controller.
     * @param K feedback gain matrices
     * @param deltaT discretization step
     * @param n0 initial time step
     */
    DiscreteStateFeedbackController(const state_vector_array_t& x_ref,
        const control_vector_array_t& uff,
        const feedback_array_t& K,
        const SCALAR deltaT,
        const int n0 = 0);

    //! copy constructor
    DiscreteStateFeedbackController(const DiscreteStateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>& other);

    //! desctructor
    virtual ~DiscreteStateFeedbackController() {}

    //! Deep cloning
    virtual DiscreteStateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override;

    //! Compute control signal
    /*!
     * Evaluate the given controller for a given state and time index
     * returns the computed control action.
     *
     * This function has to be implemented by any custom controller
     *
     * @param state current state of the system
     * @param n current time index of the system
     * @param controlAction the corresponding control action
     */
    virtual void computeControl(const state_vector_t& state, const int n, control_vector_t& controlAction) override;
};

}  // namespace core
}  // namespace ct
