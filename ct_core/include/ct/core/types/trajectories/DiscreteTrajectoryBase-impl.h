
#pragma once

namespace amira_control {

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::DiscreteTrajectoryBase(
    const InterpolationType& type)
    : time_(), data_(), interp_(type), inertialFrame_()
{
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::DiscreteTrajectoryBase(
    const tpl::TimeArray<Scalar_t>& time, const DiscreteArray_t& data, const InterpolationType& type)
    : time_(time), data_(data), interp_(type), inertialFrame_()
{
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
DERIVED DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::fromSteadyState(
    const T data, const Scalar_t time)
{
    auto timeArray = tpl::TimeArray<Scalar_t>(1, time);
    auto dataArray = DiscreteArray_t(1, data);
    return DERIVED(timeArray, dataArray, InterpolationType::ZOH);
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
DERIVED DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::fromSteadyState(
    const T data, const InertialFrame_t frame, const Scalar_t time)
{
    DERIVED inst = fromSteadyState(data, time);
    inst.setInertialFrame(frame);
    return inst;
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::DiscreteTrajectoryBase(
    const DiscreteArray_t& data, const Scalar_t& deltaT, const Scalar_t& t0, const InterpolationType& type)
    : time_(), data_(data), interp_(type)
{
    size_t nPoints = data.size();
    time_ = tpl::TimeArray<Scalar_t>(linspace<TimeArray>(t0, t0 + (nPoints - 1) * deltaT, nPoints));
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::DiscreteTrajectoryBase(
    const DiscreteTrajectoryBase_t& other)
    : time_(other.time_), data_(other.data_), interp_(other.interp_.getInterpolationType()),
      inertialFrame_(other.inertialFrame_)
{
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::DiscreteTrajectoryBase(
    DiscreteTrajectoryBase_t& other, const size_t startIndex, const size_t endIndex)
    : time_(), data_(), interp_(other.interp_.getInterpolationType())
{
    tpl::TimeArray<Scalar_t> time_temp;
    DiscreteArray_t data_temp;

    for (size_t i = startIndex; i <= endIndex; i++) {
        time_temp.push_back(other.time_[i]);
        data_temp.push_back(other.data_[i]);
    }

    time_ = time_temp;
    data_ = data_temp;
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
void DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::setData(const DiscreteArray_t& data)
{
    data_ = data;
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
void DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::setInterpolationType(
    const InterpolationType& type)
{
    interp_.changeInterpolationType(type);
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
auto DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::getInterpolation()
    -> Interpolation<T, alloc_type, Scalar_t>&
{
    return interp_;
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
auto DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::getInterpolation() const
    -> const Interpolation<T, alloc_type, Scalar_t>&
{
    return interp_;
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
void DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::setTime(const tpl::TimeArray<Scalar_t>& time)
{
    time_ = time;
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
void DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::shiftTime(const Scalar_t& dt)
{
    std::for_each(time_.begin(), time_.end(), [&](Scalar_t& val) { val -= dt; });
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
T DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::eval(const Scalar_t& evalTime)
{
    return static_cast<DERIVED*>(this)->eval_specialized(evalTime);
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
size_t DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::size() const
{
    if (data_.size() != time_.size()) {
        std::cout << "DiscreteTrajectoryBase: data size:" << data_.size() << std::endl;
        std::cout << "DiscreteTrajectoryBase: time size:" << time_.size() << std::endl;
        throw std::runtime_error("DiscreteTrajectoryBase: size inconsistent.");
    }

    return this->data_.size();
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
T& DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::front()
{
    return data_.front();
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
const T& DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::front() const
{
    return data_.front();
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
T& DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::back()
{
    return data_.back();
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
const T& DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::back() const
{
    return data_.back();
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
T& DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::operator[](const size_t i)
{
    return data_[i];
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
const T& DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::operator[](const size_t i) const
{
    return data_[i];
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
auto DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::startTime() const -> const Scalar_t
{
    return time_.front();
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
auto DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::finalTime() const -> const Scalar_t
{
    return time_.back();
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
auto DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::duration() const -> const Scalar_t
{
    return time_.back() - time_.front();
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
void DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::push_back(
    const T& data, const Scalar_t& time, const bool timeIsAbsolute)
{
    if (timeIsAbsolute)
        time_.push_back(time);
    else
        time_.push_back(time + time_.back());

    data_.push_back(data);
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
void DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::pop_back()
{
    time_.pop_back();
    data_.pop_back();
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
void DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::eraseFront(
    const size_t& N, const Scalar_t& dt)
{
    time_.erase(time_.begin(), time_.begin() + N);
    data_.erase(data_.begin(), data_.begin() + N);
    shiftTime(dt);
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
void DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::clear()
{
    data_.clear();
    time_.clear();
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
void DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::swapData(DiscreteTrajectoryBase& other)
{
    time_.swap(other.time_);
    data_.swap(other.data_);
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
auto DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::operator=(const DiscreteTrajectoryBase& other)
    -> DiscreteTrajectoryBase&
{
    if (this == &other)
        return *this;

    time_ = other.time_;
    data_ = other.data_;
    inertialFrame_ = other.inertialFrame_;
    interp_.changeInterpolationType(other.interp_.getInterpolationType());

    return *this;
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
auto DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::getTimeFromIndex(const size_t& ind) const
    -> const Scalar_t&
{
    return time_[ind];
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
size_t DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::getIndexFromTime(const Scalar_t& t)
{
    return interp_.findIndex(time_, t);
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
auto DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::getDataArray() -> DiscreteArray_t&
{
    return data_;
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
auto DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::getDataArray() const -> const DiscreteArray_t&
{
    return data_;
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
auto DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::getTimeArray() -> tpl::TimeArray<Scalar_t>&
{
    return time_;
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
auto DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::getTimeArray() const
    -> const tpl::TimeArray<Scalar_t>&
{
    return time_;
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
void DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::print()
{
    assert(time_.size() == data_.size());
    std::cout << std::endl << "Inertial Frame: " << inertialFrame_ << std::endl;
    for (size_t i = 0; i < time_.size(); i++) {
        std::cout << "time: \t " << time_[i] << std::endl << "data-point: \t" << data_[i] << std::endl;
    }
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
bool DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::isApprox(
    const DiscreteTrajectoryBase_t& other, const bool verbose)
{
    // compare inertial frames
    if (inertialFrame_ != other.getInertialFrame()) {
        if (verbose)
            std::cout << "Inertial frames are different.";
        return false;
    }

    // compare sizes
    if (time_.size() != other.getTimeArray().size()) {
        if (verbose)
            std::cout << "Time sizes are " << time_.size() << " and " << other.getTimeArray().size() << std::endl;
        return false;
    }
    if (data_.size() != other.getDataArray().size()) {
        if (verbose)
            std::cout << "Data sizes are " << data_.size() << " and " << other.getDataArray().size() << std::endl;
        return false;
    }
    // compare values
    if (interp_.getInterpolationType() != (other.getInterpolation().getInterpolationType())) {
        if (verbose)
            std::cout << "Interpol. types are " << interp_.getInterpolationType() << " and "
                      << other.getInterpolation().getInterpolationType() << std::endl;
        return false;
    }

    for (size_t i = 0; i < time_.size(); i++) {
        if (time_[i] != other.getTimeArray()[i]) {
            if (verbose)
                std::cout << "Times are " << time_[i] << " and " << other.getTimeArray()[i] << std::endl;
            return false;
        }
    }

    for (size_t i = 0; i < data_.size(); i++) {
        if (!((getDataArray()[i]).isApprox(other.getDataArray()[i]))) {
            if (verbose)
                std::cout << "Data are " << getDataArray()[i] << " and " << other.getDataArray()[i] << std::endl;
            return false;
        }
    }

    return true;
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
auto DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::getInertialFrame() const
    -> const InertialFrame_t
{
    return inertialFrame_;
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
void DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::setInertialFrame(
    const InertialFrame_t inertialFrame)
{
    inertialFrame_ = inertialFrame;
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
std::shared_ptr<DERIVED> DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::ptrFromSteadyState(
    const T data, const Scalar_t time)
{
    return std::shared_ptr<DERIVED>(new DERIVED(fromSteadyState(data, time)));
}

template <class DERIVED, class SCALAR, class T, class ALLOC, class INERTIAL_FRAME_T>
std::shared_ptr<DERIVED> DiscreteTrajectoryBase<DERIVED, SCALAR, T, ALLOC, INERTIAL_FRAME_T>::ptrFromSteadyState(
    const T data, const InertialFrame_t frame, const Scalar_t time)
{
    return std::shared_ptr<DERIVED>(new DERIVED(fromSteadyState(data, frame, time)));
}

} /* namespace amira_control */
