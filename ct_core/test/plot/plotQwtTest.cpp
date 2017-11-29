/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/
#include <ct/core/core.h>
#include <ct/core/plot/plotQwt.h>

using namespace ct::core::plotQwt;

int main(int argc, char* argv[])
{
    FigurePtr fig = createFigure();

    std::vector<double> y = {1, 3, 4, -1, 2, 4, 5};

    fig->plot(y);

    fig->draw();

    render();

    return 1;
}
