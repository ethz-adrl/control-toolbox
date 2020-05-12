/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "Controller.h"
#include <ct/core/systems/System.h>

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
template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
class StateFeedbackController : public Controller<MANIFOLD, CONTROL_DIM, TIME_T>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr size_t STATE_DIM = MANIFOLD::TangentDim;
    using SCALAR = typename MANIFOLD::Scalar;

    typedef Controller<MANIFOLD, CONTROL_DIM, TIME_T> Base;
    typedef typename Base::control_vector_t control_vector_t;
    using Time_t = typename Base::Time_t;

    typedef ControlVectorArray<CONTROL_DIM, SCALAR> control_vector_array_t;
    typedef FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR> feedback_array_t;

    StateFeedbackController();
    //! copy constructor
    StateFeedbackController(const StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>& other);

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
    StateFeedbackController(const DiscreteArray<MANIFOLD>& x_ref,
        const control_vector_array_t& uff,
        const feedback_array_t& K,
        const SCALAR deltaT,
        const SCALAR t0 = 0.0,
        const InterpolationType& intType = ZOH);

    StateFeedbackController(const MANIFOLD& x_ref,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& uff,
        const ct::core::FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>& K);

    //! destructor
    virtual ~StateFeedbackController();

    //! deep cloning
    StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>* clone() const override;

    template <ct::core::TIME_TYPE T = TIME_T>
    typename std::enable_if<T == ct::core::CONTINUOUS_TIME>::type computeControl_specialized(const MANIFOLD& state,
        const Time_t& t,
        control_vector_t& u)
    {
        u = uff_.eval(t) + K_.eval(t) * (state - x_ref_.eval(t));  // TODO: move to impl
    }

    template <ct::core::TIME_TYPE T = TIME_T>
    typename std::enable_if<T == ct::core::DISCRETE_TIME>::type computeControl_specialized(const MANIFOLD& state,
        const Time_t& n,
        control_vector_t& u)
    {
        u = uff_[n] + K_[n] * (state - x_ref_[n]);  // TODO: move to impl
    }

    //! computes the control action in the continuous case
    /*!
     * evaluates the controller using interpolation where required using interpolation
     * @param state current state
     * @param t current time
     * @param controlAction resulting control action
     */
    virtual void computeControl(const MANIFOLD& state, const Time_t& tn, control_vector_t& controlAction) override;

    //! updates the controller
    /*!
     * sets a new feedforward and feedback controller
     * @param uff feedforward control action
     * @param K feedback controller
     * @param times discretization times
     */
    void update(const DiscreteArray<MANIFOLD>& x_ref,
        const DiscreteArray<control_vector_t>& uff,
        const DiscreteArray<FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>>& K,
        const tpl::TimeArray<SCALAR>& t);

    //! get reference state vector array (without timings)
    const DiscreteArray<MANIFOLD>& x_ref() const;

    //! get feedforward array (without timings)
    const DiscreteArray<control_vector_t>& uff() const;

    //! get feedback array (without timings)
    const DiscreteArray<FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>>& K() const;

    //! get time array
    const tpl::TimeArray<SCALAR>& time() const;

    //! get a reference to the feedforward trajectory
    DiscreteTrajectory<MANIFOLD>& getReferenceStateTrajectory();

    //! get a reference to the feedforward trajectory
    const DiscreteTrajectory<MANIFOLD>& getReferenceStateTrajectory() const;

    //! get a reference to the feedforward trajectory
    ControlTrajectory<CONTROL_DIM, SCALAR>& getFeedforwardTrajectory();

    //! get a reference to the feedforward trajectory
    const ControlTrajectory<CONTROL_DIM, SCALAR>& getFeedforwardTrajectory() const;

    //! get a reference to the feedback trajectory
    FeedbackTrajectory<STATE_DIM, CONTROL_DIM, SCALAR>& getFeedbackTrajectory();

    //! get a reference to the feedback trajectory
    const FeedbackTrajectory<STATE_DIM, CONTROL_DIM, SCALAR>& getFeedbackTrajectory() const;

    //!  extracts a physically meaningful control trajectory from the given state-feedback law and a reference state trajectory
    void extractControlTrajectory(const DiscreteTrajectory<MANIFOLD>& x_traj,
        ControlTrajectory<CONTROL_DIM, SCALAR>& u_traj);

protected:
    DiscreteTrajectory<MANIFOLD> x_ref_;                    //! state reference trajectory
    ControlTrajectory<CONTROL_DIM, SCALAR> uff_;            //! feedforward control trajectory
    FeedbackTrajectory<STATE_DIM, CONTROL_DIM, SCALAR> K_;  //! feedback control trajectory
};


}  // namespace core
}  // namespace ct
