/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

template <size_t STATE_DIM, size_t CONTROL_DIM>
void plotResultsOscillator(const ct::core::StateVectorArray<STATE_DIM>& stateArray,
    const ct::core::ControlVectorArray<CONTROL_DIM>& controlArray,
    const ct::core::TimeArray& timeArray)
{
#ifdef PLOTTING_ENABLED

    using namespace ct::core;

    try
    {
        plot::ion();
        plot::figure();

        if (timeArray.size() != stateArray.size())
        {
            std::cout << timeArray.size() << std::endl;
            std::cout << stateArray.size() << std::endl;
            throw std::runtime_error("Cannot plot data, x and t not equal length");
        }

        std::vector<double> position;
        std::vector<double> velocity;
        std::vector<double> time_state;
        for (size_t j = 0; j < stateArray.size(); j++)
        {
            position.push_back(stateArray[j](0));
            velocity.push_back(stateArray[j](1));
            time_state.push_back(timeArray[j]);
        }

        std::vector<double> control;
        std::vector<double> time_control;
        for (size_t j = 0; j < controlArray.size(); j++)
        {
            control.push_back(controlArray[j](0));
            time_control.push_back(timeArray[j]);
        }

        plot::subplot(3, 1, 1);
        plot::plot(time_state, position);
        plot::title("position");

        plot::subplot(3, 1, 2);
        plot::plot(time_state, velocity);
        plot::title("velocity");

        plot::subplot(3, 1, 3);
        plot::plot(time_control, control);
        plot::title("control");

        plot::show();
    } catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
#else
    std::cout << "Plotting is disabled." << std::endl;
#endif
}
