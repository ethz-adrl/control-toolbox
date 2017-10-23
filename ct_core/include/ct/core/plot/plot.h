// Derived from :
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
//
// Derived from work of Benno Evers licensed:
//
// The MIT License (MIT)
//
// Copyright (c) 2014 Benno Evers
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <vector>
#include <map>
#include <numeric>
#include <stdexcept>
#include <iostream>
#include <initializer_list>
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace ct {
namespace core {
namespace plot {


//! Enable interactive mode.
bool ion();

//! Create a new figure.
bool figure(std::string i = "");

//! Histogram.
bool hist(const Eigen::Ref<const Eigen::VectorXd>& x, const double bins = 10, const std::string histtype = "bar");

//! Every row of X is the data for a box.
bool boxplot(const Eigen::Ref<const Eigen::MatrixXd>& x, const std::vector<std::string>& labels);
bool boxplot(const Eigen::Ref<const Eigen::MatrixXd>& x, std::initializer_list<const std::string> labels);
bool boxplot(const Eigen::Ref<const Eigen::MatrixXd>& x);

//! Create a subplot.
bool subplot(const size_t nrows, const size_t ncols, const size_t plot_number);

//! Create an x/y plot with properties as map.
bool plot(const Eigen::Ref<const Eigen::MatrixXd>& x,
    const Eigen::Ref<const Eigen::MatrixXd>& y,
    const std::map<std::string, std::string>& keywords);

//! Create an x/y plot with properties in string.
bool plot(const Eigen::Ref<const Eigen::MatrixXd>& x,
    const Eigen::Ref<const Eigen::MatrixXd>& y,
    const std::string& s = "");

//! Create an x/y plot with name as label.
bool labelPlot(const std::string& name,
    const Eigen::Ref<const Eigen::MatrixXd>& x,
    const Eigen::Ref<const Eigen::MatrixXd>& y,
    const std::string& format = "");
bool labelPlot(const std::string& name, const Eigen::Ref<const Eigen::MatrixXd>& y, const std::string& format = "");

bool plot(const Eigen::Ref<const Eigen::MatrixXd>& x, const std::string& format = "");

// -----------------------------------------------------------------------------
//! @name std::vector wrappers.
//! @{

template <typename ALLOC>
bool plot(const std::vector<double, ALLOC>& y, const std::string& format = "");

template <typename ALLOC, typename ALLOC2>
bool plot(const std::vector<double, ALLOC>& x,
    const std::vector<double, ALLOC2>& y,
    const std::map<std::string, std::string>& keywords);

bool plot(const std::vector<double>& x,
    const Eigen::Ref<const Eigen::MatrixXd>& y,
    const std::map<std::string, std::string>& keywords);

template <typename ALLOC, typename ALLOC2>
bool plot(const std::vector<double, ALLOC>& x, const std::vector<double, ALLOC2>& y, const std::string& s = "");

bool plot(const std::vector<double>& x, const Eigen::Ref<const Eigen::MatrixXd>& y, const std::string& s = "");

bool labelPlot(const std::string& name,
    const std::vector<double>& x,
    const std::vector<double>& y,
    const std::string& format = "");

bool labelPlot(const std::string& name,
    const std::vector<double>& x,
    const Eigen::Ref<const Eigen::MatrixXd>& y,
    const std::string& format = "");
//! @}

// -----------------------------------------------------------------------------
//! @name Plot settings.
//! @{
void legend();

void ylim(double min, double max);

void xlim(double xmin, double xmax);

void title(const std::string& titlestr);

void axis(const std::string& axisstr);

void xlabel(const std::string& str);

void ylabel(const std::string& str);

void grid(bool flag);

void show(bool block = true);

void save(const std::string& filename);
//! @}

void warn();

#include "plot-impl.h"
}
}
}
