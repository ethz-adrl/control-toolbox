/*
 * plotQwtTest.cpp
 *
 *  Created on: Mar 17, 2017
 *      Author: neunertm
 */

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

