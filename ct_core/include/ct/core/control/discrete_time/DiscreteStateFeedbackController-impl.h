
namespace ct {
namespace core {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
DiscreteStateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::DiscreteStateFeedbackController(
    const state_vector_array_t& x_ref,
    const control_vector_array_t& uff,
    const feedback_array_t& K,
    const SCALAR deltaT,
    const int n0)
    : StateFeedbackBase(x_ref, uff, K, deltaT, deltaT * n0, ZOH)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
DiscreteStateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::DiscreteStateFeedbackController(
    const DiscreteStateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>& other)
    : StateFeedbackBase(other)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
DiscreteStateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>*
DiscreteStateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::clone() const
{
    return new DiscreteStateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void DiscreteStateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::computeControl(const state_vector_t& state,
    const int n,
    control_vector_t& controlAction)
{
    controlAction = this->uff_[n] + this->K_[n] * (state - this->x_ref_[n]);
}

}  // namespace core
}  // namespace ct
