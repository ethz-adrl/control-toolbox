/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

void plotResults(const ct::core::StateVectorArray<2>& stateArray,
    const ct::core::ControlVectorArray<1>& controlArray,
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

        std::vector<std::vector<double>> states;
        std::vector<double> time_state;
        std::vector<double> constraint;
        for (size_t k = 0; k < 2; k++)
        {
            states.push_back(std::vector<double>());
        }

        for (size_t j = 0; j < stateArray.size(); j++)
        {
            for (size_t k = 0; k < 2; k++)
            {
                states[k].push_back(stateArray[j](k));
            }
            time_state.push_back(timeArray[j]);
            constraint.push_back(stateArray[j](0) + stateArray[j](1));
        }

        std::vector<double> control;
        std::vector<double> time_control;
        for (size_t j = 0; j < controlArray.size(); j++)
        {
            control.push_back(controlArray[j](0));
            time_control.push_back(timeArray[j]);
        }

        for (size_t k = 0; k < 2; k++)
        {
            plot::subplot(2, 1, k + 1);
            plot::plot(time_state, states[k]);
            plot::title("x(" + std::to_string(k) + ")");
        }

        plot::figure();
        plot::plot(time_state, constraint);
        plot::title("Constraint x(0) + x(1)");


        plot::figure();
        plot::plot(time_control, control);
        plot::title("Control");

        plot::show();
    } catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
#else
    std::cout << "Plotting is disabled." << std::endl;
#endif
}