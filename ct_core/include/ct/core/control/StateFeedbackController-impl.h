/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

namespace ct {
namespace core {

template <typename MANIFOLD, size_t CONTROL_DIM, TIME_TYPE TIME_T>
StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::StateFeedbackController() : Base()
{
}

template <typename MANIFOLD, size_t CONTROL_DIM, TIME_TYPE TIME_T>
StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::~StateFeedbackController()
{
}

template <typename MANIFOLD, size_t CONTROL_DIM, TIME_TYPE TIME_T>
StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::StateFeedbackController(const MANIFOLD& x_ref,
    const ct::core::ControlVector<CONTROL_DIM, SCALAR>& uff,
    const ct::core::FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>& K)
    : StateFeedbackController(ct::core::DiscreteArray<MANIFOLD>(1, x_ref),
          control_vector_array_t(1, uff),
          FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR>(1, K),
          1.0,
          0,
          ZOH)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM, TIME_TYPE TIME_T>
StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::StateFeedbackController(
    const StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>& other)
    : Base(), x_ref_(other.x_ref_), uff_(other.uff_), K_(other.K_)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM, TIME_TYPE TIME_T>
StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::StateFeedbackController(const DiscreteArray<MANIFOLD>& x_ref,
    const ControlVectorArray<CONTROL_DIM, SCALAR>& uff,
    const FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR>& K,
    const SCALAR deltaT,
    const SCALAR t0,
    const InterpolationType& intType)
    : x_ref_(x_ref, deltaT, t0, intType), uff_(uff, deltaT, t0, intType), K_(K, deltaT, t0, intType)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM, TIME_TYPE TIME_T>
void StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::update(const DiscreteArray<MANIFOLD>& x_ref,
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

template <typename MANIFOLD, size_t CONTROL_DIM, TIME_TYPE TIME_T>
void StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::computeControl(const MANIFOLD& state,
    const Time_t& tn,
    control_vector_t& u)
{
    computeControl_specialized(state, tn, u);
}

template <typename MANIFOLD, size_t CONTROL_DIM, TIME_TYPE TIME_T>
StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>* StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::clone()
    const
{
    return new StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>(*this);
}

template <typename MANIFOLD, size_t CONTROL_DIM, TIME_TYPE TIME_T>
const DiscreteArray<MANIFOLD>& StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::x_ref() const
{
    return x_ref_.getDataArray();
}

template <typename MANIFOLD, size_t CONTROL_DIM, TIME_TYPE TIME_T>
const DiscreteArray<typename StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::control_vector_t>&
StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::uff() const
{
    return uff_.getDataArray();
}

template <typename MANIFOLD, size_t CONTROL_DIM, TIME_TYPE TIME_T>
auto StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::K() const
    -> const DiscreteArray<FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>>&
{
    return K_.getDataArray();
}

template <typename MANIFOLD, size_t CONTROL_DIM, TIME_TYPE TIME_T>
auto StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::time() const -> const tpl::TimeArray<SCALAR>&
{
    return x_ref_.getTimeArray();
}

template <typename MANIFOLD, size_t CONTROL_DIM, TIME_TYPE TIME_T>
DiscreteTrajectory<MANIFOLD>& StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::getReferenceStateTrajectory()
{
    return x_ref_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, TIME_TYPE TIME_T>
auto StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::getReferenceStateTrajectory() const
    -> const DiscreteTrajectory<MANIFOLD>&
{
    return x_ref_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, TIME_TYPE TIME_T>
auto StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::getFeedforwardTrajectory()
    -> ControlTrajectory<CONTROL_DIM, SCALAR>&
{
    return uff_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, TIME_TYPE TIME_T>
auto StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::getFeedforwardTrajectory() const
    -> const ControlTrajectory<CONTROL_DIM, SCALAR>&
{
    return uff_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, TIME_TYPE TIME_T>
auto StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::getFeedbackTrajectory()
    -> FeedbackTrajectory<STATE_DIM, CONTROL_DIM, SCALAR>&
{
    return K_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, TIME_TYPE TIME_T>
auto StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::getFeedbackTrajectory() const
    -> const FeedbackTrajectory<STATE_DIM, CONTROL_DIM, SCALAR>&
{
    return K_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, TIME_TYPE TIME_T>
void StateFeedbackController<MANIFOLD, CONTROL_DIM, TIME_T>::extractControlTrajectory(
    const DiscreteTrajectory<MANIFOLD>& x_traj,
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
