/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#include <thread>
#include <random>
#include <chrono>

// show verbose output
#define DEBUG_PRINT_TIMEKEEPER

#include <ct/optcon/mpc/MpcTimeKeeper.h>
#include <ct/optcon/mpc/MpcSettings.h>
#include <ct/optcon/mpc/timehorizon/MpcTimeHorizon.h>


/**
 * This executable is not a unit test, but can be employed for manual debugging of the MPC - timings.
 */

int main(int argc, char **argv)
{
    ct::optcon::mpc_settings settings;
    settings.stateForwardIntegration_ = true;
    settings.measureDelay_ = true;
    settings.delayMeasurementMultiplier_ = 1.0;

    // for random delays
    std::mt19937_64 eng{std::random_device{}()};
    std::uniform_int_distribution<> dist{70, 120};

    // the start time
    auto start_time = std::chrono::high_resolution_clock::now();

    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_init;
    std::chrono::time_point<std::chrono::high_resolution_clock> stop_time_init;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_int;
    std::chrono::time_point<std::chrono::high_resolution_clock> stop_time_int;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_opt;
    std::chrono::time_point<std::chrono::high_resolution_clock> stop_time_opt;

    ct::core::Time dt_forward_simulated;
    ct::core::Time currentT = 1.3450;  // a random value


    auto tstrat = std::shared_ptr<ct::optcon::MpcTimeHorizon>(new ct::optcon::MpcTimeHorizon(settings, currentT));

    ct::optcon::MpcTimeKeeper timeKeeper(tstrat, settings);
    bool firstRun_ = true;

    for (size_t i = 0; i < 40; i++)
    {
        std::cout << std::endl;

        start_time_init = std::chrono::high_resolution_clock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds{dist(eng)});  // random delay, e.g. for communication
        stop_time_init = std::chrono::high_resolution_clock::now();


        auto current_time = std::chrono::high_resolution_clock::now();
        ct::core::Time t =
            0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();

        if (firstRun_)
        {
            timeKeeper.initialize();
        }


        timeKeeper.startDelayMeasurement();

        ct::core::Time t_forward_start;
        ct::core::Time t_forward_stop;
        ct::core::Time newT;
        timeKeeper.computeNewTimings(currentT, newT, t_forward_start, t_forward_stop);


        if (!firstRun_)
        {
            // perform forward integration relative to previous controller
            // the assumption here is that the given controller starts at the time of the last call to MPC + the respective forward integration time
            if (settings.stateForwardIntegration_ == true)
            {
                start_time_int = std::chrono::high_resolution_clock::now();
                std::this_thread::sleep_for(std::chrono::milliseconds{dist(eng)});  // random delay
                stop_time_int = std::chrono::high_resolution_clock::now();
            }
        }


        // HERE we solve the optimization problem
        start_time_opt = std::chrono::high_resolution_clock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds{dist(eng)});  // random delay
        stop_time_opt = std::chrono::high_resolution_clock::now();


        // most likely, planning took longer than the pre-integration estimated
        ct::core::Time dtp = timeKeeper.timeSincePreviousSuccessfulSolve();


        timeKeeper.stopDelayMeasurement();


        currentT = newT;


        if (firstRun_)
            firstRun_ = false;


        double delta_t_init =
            0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(stop_time_init - start_time_init).count();
        double delta_t_int =
            0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(stop_time_int - start_time_int).count();
        double delta_t_opt =
            0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(stop_time_opt - start_time_opt).count();

        std::cout << "init delay.  actually took: " << delta_t_init << std::endl;
        std::cout << "Preintegr.   actually took: " << delta_t_int << std::endl;
        std::cout << "Optimization actually took: " << delta_t_opt << std::endl;
        std::cout << "Pre-int + Opt took : 	 " << delta_t_int + delta_t_opt << std::endl;
        std::cout << "Total delay: 	 " << delta_t_int + delta_t_opt + delta_t_init << std::endl;
    }
}
