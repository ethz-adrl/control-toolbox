/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#pragma once

#include <time.h>
#include <random>

namespace ct {
namespace core {

//! Gaussian noise generator
/*!
 * This class generates random Gaussian noise given a mean and a distribtion. It can
 * either create a single (pseudo) random variable or an entire vector.
 *
 * Unit test \ref NoiseTest.cpp illustrates the use of GaussianNoise
 */

class GaussianNoise
{
public:

	//! Standard constructor
	/*!
	 * @param mean the mean of the Gaussian distribution
	 * @param standardDeviation the standard deviation of the distribution
	 */
	GaussianNoise(double mean=0.0, double standardDeviation=1.0) :
		rd_(),
		eng_(rd_()),
		distr_(mean, standardDeviation)
	{}

	//! Scalar generator
	/*!
	 *  generates a single scalar random variable
	 * @return random variable
	 */
	double operator()() { return distr_(eng_); }

	//! Vector generator
	/*!
	 * All entries in the vector are separately generated random variables
	 * @return vector of random variables
	 */
	template <size_t size>
	Eigen::Matrix<double, size, 1> gen() {
		Eigen::Matrix<double, size, 1> noise;

		for (size_t i=0; i<size; i++)
		{
			noise(i) = distr_(eng_);
		}

		return noise;
	}

	//! adds Gaussian noise to a single scalar variable
	void noisify(double& value) { value += this->operator()(); }

	//! adds Gaussian noise to a vector
	/*!
	 * Different random variables are used to perturb each entry of the vector
	 * @param value the vector to perturb
	 */
	template <size_t size>
	void noisify(Eigen::Matrix<double, size, 1>& value) {
		value += this->gen<size>();
	}


private:
	std::random_device rd_;
	std::mt19937 eng_;
	std::normal_distribution<> distr_;
};


} // namespace ct
} // namespace core

