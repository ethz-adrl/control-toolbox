/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

namespace ct {
namespace core {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::StateFeedbackController(
    const StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>& other)
    : ContinuousBase(), DiscreteBase(), x_ref_(other.x_ref_), uff_(other.uff_), K_(other.K_)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::StateFeedbackController(
    const StateVectorArray<STATE_DIM, SCALAR>& x_ref,
    const ControlVectorArray<CONTROL_DIM, SCALAR>& uff,
    const FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR>& K,
    const SCALAR deltaT,
    const SCALAR t0,
    const InterpolationType& intType)
    : x_ref_(x_ref, deltaT, t0, intType), uff_(uff, deltaT, t0, intType), K_(K, deltaT, t0, intType)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::computeControl(const state_vector_t& state,
    const SCALAR& t,
    control_vector_t& controlAction)
{
    controlAction = uff_.eval(t) + K_.eval(t) * (state - x_ref_.eval(t));
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::computeControl(const state_vector_t& state,
    const int n,
    control_vector_t& controlAction)
{
    controlAction = uff_[n] + K_[n] * (state - x_ref_[n]);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::update(const DiscreteArray<state_vector_t>& x_ref,
    const DiscreteArray<control_vector_t>& uff,
    const DiscreteArray<FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>>& K,
    const tpl::TimeArray<SCALAR>& t)
{
    tpl::TimeArray<SCALAR> tshort = t;
    tshort.pop_back();  // todo: the copying here is not optimal

    if (K.size() != tshort.size())
        throw std::runtime_error("StateFeedbackController.h : K.size() != tshort.size()");
    if (uff.size() != tshort.size())
        throw std::runtime_error("StateFeedbackController.h : uff.size() != tshort.size()");
    if (x_ref.size() != t.size())
        throw std::runtime_error("StateFeedbackController.h : x_ref.size() != t.size()");

    x_ref_.setData(x_ref), x_ref_.setTime(t), uff_.setData(uff);
    uff_.setTime(tshort);
    K_.setData(K);
    K_.setTime(tshort);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>*
StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::clone() const
{
    return new StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const DiscreteArray<typename StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t>&
StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::x_ref() const
{
    return x_ref_.getDataArray();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const DiscreteArray<typename StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::control_vector_t>&
StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::uff() const
{
    return uff_.getDataArray();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const DiscreteArray<FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>>&
StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::K() const
{
    return K_.getDataArray();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const tpl::TimeArray<SCALAR>& StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::time() const
{
    return x_ref_.getTimeArray();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateTrajectory<STATE_DIM, SCALAR>&
StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::getReferenceStateTrajectory()
{
    return x_ref_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const StateTrajectory<STATE_DIM, SCALAR>&
StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::getReferenceStateTrajectory() const
{
    return x_ref_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ControlTrajectory<CONTROL_DIM, SCALAR>&
StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::getFeedforwardTrajectory()
{
    return uff_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const ControlTrajectory<CONTROL_DIM, SCALAR>&
StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::getFeedforwardTrajectory() const
{
    return uff_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
FeedbackTrajectory<STATE_DIM, CONTROL_DIM, SCALAR>&
StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::getFeedbackTrajectory()
{
    return K_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const FeedbackTrajectory<STATE_DIM, CONTROL_DIM, SCALAR>&
StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::getFeedbackTrajectory() const
{
    return K_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::extractControlTrajectory(
    const StateTrajectory<STATE_DIM, SCALAR>& x_traj,
    ControlTrajectory<CONTROL_DIM, SCALAR>& u_traj)
{
    u_traj.clear();

    for (size_t i = 0; i < x_traj.size() - 1; i++)
    {
        u_traj.push_back(uff_[i] + K_[i] * (x_traj[i] - x_ref_[i]), x_traj.getTimeFromIndex(i), true);
    }
}

}  // namespace core
}  // namespace ct
