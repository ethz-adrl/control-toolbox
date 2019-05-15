/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

template <size_t NJOINTS>
void plotResultsTrajectories(const ct::core::StateTrajectory<2 * NJOINTS>& stateTraj)
{
#ifdef PLOTTING_ENABLED

    using namespace ct::core;

    try
    {
        plot::ion();
        plot::figure();

        for (size_t jointId = 0; jointId < NJOINTS; jointId++)
        {
            std::vector<double> position;
            std::vector<double> velocity;
            std::vector<double> time_state;
            for (size_t j = 0; j < stateTraj.getDataArray().size(); j++)
            {
                position.push_back(stateTraj.getDataArray()[j](jointId));
                velocity.push_back(stateTraj.getDataArray()[j](jointId + 6));
                time_state.push_back(stateTraj.getTimeArray()[j]);
            }


            plot::subplot(NJOINTS, 2, 2 * jointId + 1);
            plot::plot(time_state, position);
            plot::title("position");

            plot::subplot(NJOINTS, 2, 2 * jointId + 2);
            plot::plot(time_state, velocity);
            plot::title("velocity");
        }
        plot::show();
    } catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
#else
    std::cout << "Plotting is disabled." << std::endl;
#endif
}
