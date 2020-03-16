/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {
//! Declaring Switched alias such that we can write Switched<System>
template <class T, class Alloc = Eigen::aligned_allocator<T>>
using Switched = std::vector<T, Alloc>;

//! Describes a switch between phases
template <class Phase, typename Time>
struct SwitchEvent
{
    Phase pre_phase;
    Phase post_phase;
    Time switch_time;
};

//! Describes a Phase sequence with timing
/*!
 *  Each phase of the sequence has a start time, and end time
 *  Each event has a pre & post phase + switching time
 *
 *  Example:
 *  The following illustrates a sequence of 3 phases
 *  + ------- + ------- + ------- +
 *  t0   p0   t1   p1   t2   p2   t3
 *
 *  It contains 4 times to define the sequence.
 *  Two of those are switching times: t1 and t2
 *  There are two switching events: {p0, p1, t1} and {p1, p2, t2};
 *
 */
template <class Phase, typename Time>
class PhaseSequence
{
public:
    typedef std::vector<Phase> PhaseSchedule_t;
    typedef std::vector<Time> TimeSchedule_t;

    //! Construct empty sequence (with default start time)
    PhaseSequence(const Time& start_time = 0) { time_schedule_.push_back(start_time); }
    //! Destructor
    ~PhaseSequence() {}
    /// @brief add a phase with duration
    void addPhase(Phase phase, Time duration)
    {
        phase_schedule_.push_back(std::move(phase));
        time_schedule_.emplace_back(time_schedule_.back() + duration);
    }
    /// @brief get number of phases
    std::size_t getNumPhases() const { return phase_schedule_.size(); }
    /// @brief get number of switches
    std::size_t getNumSwitches() const { return getNumPhases() - 1; }
    /// @brief get sequence total duration
    Time getTotalDuration() const { return time_schedule_.back() - time_schedule_.front(); }
    /// @brief get start time from sequence index
    Time getStartTimeFromIdx(std::size_t idx) const { return time_schedule_[idx]; }
    /// @brief get end time from sequence index
    Time getEndTimeFromIdx(std::size_t idx) const { return time_schedule_[idx + 1]; }
    /// @brief get phase pointer from sequence index
    Phase getPhaseFromIdx(std::size_t idx) const { return phase_schedule_[idx]; }
    /// @brief get phase pointer from time
    Phase getPhaseFromTime(Time time) const { return getPhaseFromIdx(getIdxFromTime(time)); }
    /// @brief get First phase pointer
    Phase getFirstPhase() const { return phase_schedule_.front(); };
    /// @brief get Final phase pointer
    Phase getFinalPhase() const { return phase_schedule_.back(); };
    /// @brief get next switch event from sequence index
    SwitchEvent<Phase, Time> getSwitchEventFromIdx(std::size_t idx) const
    {
        return {getPhaseFromIdx(idx), getPhaseFromIdx(idx + 1), getEndTimeFromIdx(idx)};
    }
    /// @brief get next switch event from time
    SwitchEvent<Phase, Time> getSwitchEventFromTime(Time time) const
    {
        return getSwitchEventFromIdx(getIdxFromTime(time));
    }
    /// @brief get sequence index from time
    std::size_t getIdxFromTime(Time time) const
    {
        // Finds pointer to first element less or equal to time
        // i.e. it returns the index for the phase with time in [t_start, t_end)
        // times outside the vector are mapped to the first and last phase
        if (time < time_schedule_.front())
        {
            return 0;
        }
        else if (time >= time_schedule_.back())
        {
            return getNumPhases() - 1;
        }
        else
        {
            auto up = std::upper_bound(time_schedule_.begin(), time_schedule_.end(), time);
            return up - time_schedule_.begin() - 1;
        }
    }

private:
    PhaseSchedule_t phase_schedule_;
    TimeSchedule_t time_schedule_;
};

using ContinuousModeSequence = PhaseSequence<std::size_t, double>;
using ContinuousModeSwitch = SwitchEvent<std::size_t, double>;

using DiscreteModeSequence = PhaseSequence<std::size_t, int>;
using DiscreteModeSwitch = SwitchEvent<std::size_t, int>;

}  // namespace core
}  // namespace ct