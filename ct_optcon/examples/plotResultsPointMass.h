/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

template <size_t STATE_DIM, size_t CONTROL_DIM>
void plotResultsPointMass(const ct::core::StateVectorArray<STATE_DIM>& stateArray,
    const ct::core::ControlVectorArray<CONTROL_DIM>& controlArray,
    const ct::core::TimeArray& timeArray, const double r, const double x_obst, const double y_obst)
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

        std::vector<double> position_x;
        std::vector<double> position_y;
        std::vector<double> time_state;
        for (size_t j = 0; j < stateArray.size(); j++)
        {
            position_x.push_back(stateArray[j](0));
            position_y.push_back(stateArray[j](1));
            time_state.push_back(timeArray[j]);
        }

        std::vector<double> control_x;
        std::vector<double> control_y;
        std::vector<double> time_control;
        for (size_t j = 0; j < controlArray.size(); j++)
        {
            control_x.push_back(controlArray[j](0));
            control_y.push_back(controlArray[j](1));
            time_control.push_back(timeArray[j]);
        }

        Eigen::VectorXd theta = Eigen::VectorXd::LinSpaced(200, 0, 2*3.14159265359);
        Eigen::VectorXd x, y;
        x = r*theta.array().sin().matrix();
        y = r*theta.array().cos().matrix();
        x.array() = x.array() + x_obst;
        y.array() = y.array() + y_obst;

        plot::subplot(3, 1, 1);
        plot::plot(position_x, position_y);
        plot::plot(x,y);
        plot::axis("equal");
        plot::title("phase plot");

        plot::subplot(3, 1, 2);
        plot::plot(time_control, control_x);
        plot::title("control x");

        plot::subplot(3, 1, 3);
        plot::plot(time_control, control_y);
        plot::title("control y");


        plot::show();
    } catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
#else
    std::cout << "Plotting is disabled." << std::endl;
#endif
}
