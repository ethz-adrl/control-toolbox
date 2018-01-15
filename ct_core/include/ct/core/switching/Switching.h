/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
    namespace core {
        //! A general class to store mode dependent object
        /*!
         *  All modes are stored in a vector
         *  The vector index doubles as the mode number
         */
        template <T>
        class Switched {
        public:
            typedef std::vector<T> modes_t;        //<! list of modes

            //! constructor
            Switched(const modes_t& modes) : modes_(modes) {}
            //! desctructor
            ~Switched() {}

            /// @brief get base pose
            T& getMode(const std::size_t& mode_number) { return modes_[mode_number]; }
        protected:
            modes_t modes_;  //!< list of modes
        };

        //! Describes a switch between modes
        struct ModeSwitch {
            std::size_t curr_mode;
            std::size_t next_mode;
            double switch_time;
        };

        //! Describes a single phase
        struct Phase {
            std::size_t mode;
            double start_time;
            double end_time;
        };

        //! Describes a mode sequence with timing
        /*!
         *  Each phase of the sequence has a mode number, start time, and end time
         *  This is stored in a vector of modes and vector of times
         */
        class ModeSequence {
        public:
            typedef std::vector<std::size_t> mode_schedule_t;  //<! list of modes for each phase
            typedef std::vector<double> time_schedule_t;       //<! switching times including start and end

            //! Construct empty sequence (with default start time)
            ModeSequence(const double& start_time = 0.0) {
              time_schedule_.push_back(start_time);
            }
            //! Construct from sequence
            ModeSequence(const mode_schedule_t& mode_schedule, const time_schedule_t& time_schedule) :
                mode_schedule_(mode_schedule),
                time_schedule_(time_schedule){
              if (mode_schedule_.size() != (time_schedule_.size() - 1)) {
                throw std::runtime_error("Mode schedule needs be length of times schedule - 1");
              };
              if (time_schedule_.size() < 2) {
                throw std::runtime_error("time schedule needs to be longer than two");
              };
            }
            //! desctructor
            ~ModeSequence() {}

            /// @brief add a phase with mode and duration
            void addPhase(const std::size_t& mode, const double& duration) {
              mode_schedule_.push_back(mode);
              time_schedule_.emplace_back(time_schedule_.back() + duration);
            }
            /// @brief get number of phases
            std::size_t getNumPhases() { return mode_schedule_.size(); }
            /// @brief get number of switches
            std::size_t getNumSwitches() { return getNumPhases() - 1; }
            /// @brief get mode from phase index
            const std::size_t& getModeFromIdx( const std::size_t& idx) { return mode_schedule_[idx]; }
            /// @brief get start time from phase index
            const double& getStartTimeFromIdx( const std::size_t& idx) { return switch_times_[idx]; }
            /// @brief get end time from phase index
            const double& getEndTimeFromIdx( const std::size_t& idx) { return switch_times_[idx+1]; }
            /// @brief get next mode switch from phase index
            ModeSwitch getModeSwitchFromIdx( const std::size_t& idx) {
              return {getModeFromIdx(idx), getModeFromIdx(idx+1), getEndTimeFromIdx(idx)};
            }
            /// @brief get phase from phase index
            Phase getPhase ( const std::size_t& idx ){
              return {getModeFromIdx(idx), getStartTimeFromIdx(idx), getEndTimeFromIdx(idx)};
            }

        protected:
            mode_schedule_t mode_schedule_;
            time_schedule_t time_schedule_;
        };

    }  // namespace core
}  // namespace ct