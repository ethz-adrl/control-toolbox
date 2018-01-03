/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <atomic>
#include <chrono>
#include <thread>
#include <memory>

#include <ct/core/types/Time.h>
#include <ct/core/types/StateVector.h>
#include <ct/core/integration/Integrator.h>
#include <ct/core/control/continuous_time/Controller.h>

namespace ct {
namespace core {

//! A class for simulating controlled systems in a general way
/*!
 * This runs two threads - one that integrates the controlled system by applying the defined control,
 * and one that updates the control if needed.
 *
 * @tparam CONTROLLED_SYSTEM the controlled system that we wish to simulate
 */
template <class CONTROLLED_SYSTEM>
class ControlSimulator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_DIM   = CONTROLLED_SYSTEM::STATE_DIM;
    static const size_t CONTROL_DIM = CONTROLLED_SYSTEM::CONTROL_DIM;

    using SCALAR = typename CONTROLLED_SYSTEM::SCALAR;

    //! default constructor
    ControlSimulator() {}
    //! constructor
    ControlSimulator(Time sim_dt,
        Time control_dt,
        const StateVector<STATE_DIM>& x0,
        std::shared_ptr<CONTROLLED_SYSTEM> controlled_system,
        std::shared_ptr<Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller = nullptr)
        : sim_dt_(sim_dt),
          control_dt_(control_dt),
          x0_(x0),
          system_(controlled_system),
          controller_(controller),
          stop_(false)
    {
        if (sim_dt_ <= 0 || control_dt_ <= 0) throw "Step sizes must be positive.";
        if (sim_dt_ > control_dt_) throw "Simulation step must be smaller than the control step.";
        if (controller_) system_->setController(controller_);
    }

    //! copy constructor
    ControlSimulator(const ControlSimulator& arg)
        : sim_dt_(arg.sim_dt_), control_dt_(arg.control_dt_), x0_(arg.x0_), stop_(arg.stop_)
    {
        if (!arg.system_) return;
        system_ = std::shared_ptr<Controller<STATE_DIM, CONTROL_DIM, SCALAR>>(arg.system_->clone());
        if (arg.controller_)
        {
            controller_ = std::shared_ptr<Controller<STATE_DIM, CONTROL_DIM, SCALAR>>(arg.controller_->clone());
            system_->setController(controller_);
        }
    }

    //! destructor
    virtual ~ControlSimulator() { finish(); };
    //! During controller update, this method does processing before the state measurement arrives
    virtual bool prepareControllerIteration(Time sim_time) = 0;

    //! During controller update, this method does processing once the state measurement arrives
    virtual bool finishControllerIteration(Time sim_time) = 0;

    //! spawns the two threads in a nonblocking way
    void simulate(Time duration)
    {
        stop_           = false;
        sim_start_time_ = std::chrono::high_resolution_clock::now();
        x_              = x0_;
        sys_thread_     = std::thread(&ControlSimulator::simulateSystem, this, duration);
        control_thread_ = std::thread(&ControlSimulator::simulateController, this, duration);
    }

    //! waits for the simulation threads to finish
    void finish()
    {
        if (sys_thread_.joinable()) sys_thread_.join();
        if (control_thread_.joinable()) control_thread_.join();
    }

    //! stops the simulation
    void stop() { stop_ = true; }

protected:
    //! method run by the thread that simulates the system
    virtual void simulateSystem(Time duration)
    {
        Integrator<STATE_DIM> integrator(system_);
        Time sim_time;
        auto wall_time                = sim_start_time_;
        StateVector<STATE_DIM> temp_x = x_;
        // In case an integer number of steps cannot be performed
        const double residue = control_dt_ / sim_dt_ - int(control_dt_ / sim_dt_);

        while (std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - sim_start_time_).count() <
                   duration &&
               !stop_)
        {
            sim_time = std::chrono::duration<double>(wall_time - sim_start_time_).count();

            integrator.integrate_n_steps(temp_x, sim_time, int(control_dt_ / sim_dt_), sim_dt_);
            if (residue > 1e-6)
                integrator.integrate_n_steps(temp_x, sim_time + int(control_dt_ / sim_dt_) * sim_dt_, 1, residue);

            if (std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - wall_time).count() >=
                control_dt_)
                std::cerr << "Simulation running too slow. Please increase the step size!" << std::endl;
            while (std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - wall_time).count() <
                   control_dt_)
                usleep(100);
            wall_time += std::chrono::duration_cast<std::chrono::system_clock::duration>(
                std::chrono::duration<double>(control_dt_));

            state_mtx_.lock();
            x_ = temp_x;
            state_mtx_.unlock();

            policy_mtx_.lock();
            system_->setController(controller_);
            policy_mtx_.unlock();
        }
    }

    //! method run by the thread that updates the controller
    virtual void simulateController(Time duration)
    {
        Time sim_time;
        while (std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - sim_start_time_).count() <
                   duration &&
               !stop_)
        {
            sim_time =
                std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - sim_start_time_).count();
            bool success = prepareControllerIteration(sim_time);
            sim_time =
                std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - sim_start_time_).count();
            success &= finishControllerIteration(sim_time);

            if (!success) throw "Simulation failed!";
        }
    }

    Time sim_dt_;
    Time control_dt_;
    std::shared_ptr<CONTROLLED_SYSTEM> system_;
    std::shared_ptr<Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller_;
    std::chrono::time_point<std::chrono::high_resolution_clock> sim_start_time_;
    StateVector<STATE_DIM> x0_;
    StateVector<STATE_DIM> x_;
    std::thread sys_thread_;
    std::thread control_thread_;
    std::mutex state_mtx_;
    std::mutex policy_mtx_;
    std::atomic<bool> stop_;
};

}  // namespace core
}  // namespace ct
