/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <time.h>
#include <random>

namespace ct {
namespace core {

//! Uniform noise generator
/*!
 * This class generates random uniform noise given a mean and a window size. It can
 * either create a single (pseudo) random variable or an entire vector.
 *
 */
class UniformNoise
{
public:
    //! Standard constructor
    /*!
	 * @param mean the mean of the uniform distribution
	 * @param r the half-width of the distribution
	 */
    UniformNoise(const double mean = 0.0, const double r = 1.0) : rd_(), eng_(rd_()), distr_(mean - r, mean + r) {}
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
    Eigen::Matrix<double, size, 1> gen()
    {
        Eigen::Matrix<double, size, 1> noise;

        for (size_t i = 0; i < size; i++)
        {
            noise(i) = distr_(eng_);
        }

        return noise;
    }

    //! adds uniform noise to a single scalar variable
    void noisify(double& value) { value += this->operator()(); }
    //! adds uniform noise to a vector
    /*!
	 * Different random variables are used to perturb each entry of the vector
	 * @param value the vector to perturb
	 */
    template <size_t size>
    void noisify(Eigen::Matrix<double, size, 1>& value)
    {
        value += this->gen<size>();
    }


private:
    std::random_device rd_;
    std::mt19937 eng_;
    std::uniform_real_distribution<> distr_;
};


}  // namespace ct
}  // namespace core
