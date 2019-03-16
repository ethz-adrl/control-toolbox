// Derived from:
//
// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE
// FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <ct/core/core.h>

int main()
{
    // Simple:
    std::vector<double> v({1, 2, 3, 4});
    ct::core::plot::plot(v);
    ct::core::plot::show();

    // Eigen Vector Types:
    Eigen::VectorXd times(100);
    times.setLinSpaced(100, 0, 20);
    Eigen::VectorXd points(100);
    points.setRandom();

    ct::core::plot::plot(times, points);
    ct::core::plot::show();

    ct::core::plot::labelPlot("A Name", times, points);
    ct::core::plot::show();

    // enable interactive mode as of now (only tests if it doesn't crash)
    // ct::core::plot::ion();

    // subplots
    ct::core::plot::subplot(3, 1, 1);
    ct::core::plot::plot(v);
    ct::core::plot::subplot(3, 1, 2);
    ct::core::plot::plot(v);
    ct::core::plot::subplot(3, 1, 3);
    ct::core::plot::plot(v);
    ct::core::plot::show(false);


    // plot multiple curves in a single graph
    std::vector<double> w({4, 3, 2, 1});
    ct::core::plot::plot(v, "x");
    ct::core::plot::plot(w, "o");
    ct::core::plot::show();

    // Histogram
    ct::core::plot::hist(points, 3);
    ct::core::plot::show();

    // Row vectors
    Eigen::MatrixXd matrix(2, 100);
    matrix.setRandom();
    ct::core::plot::plot(matrix.row(0), matrix.row(1));
    ct::core::plot::show();

    // BoxPlot
    Eigen::MatrixXd data(2, 100);
    data.setRandom();
    ct::core::plot::figure();
    std::vector<std::string> labels = {"A", "B"};
    ct::core::plot::boxplot(data, labels);
    ct::core::plot::show();

    // BoxPlot
    data.setRandom();
    ct::core::plot::figure();
    ct::core::plot::boxplot(data, {"A", "B"});
    ct::core::plot::show();

    // Boxplot unlabelled
    data.setRandom();
    ct::core::plot::figure();
    ct::core::plot::boxplot(data);
    ct::core::plot::show();
}
