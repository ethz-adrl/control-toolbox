/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "DiscreteTrajectory.h"

namespace ct {
namespace core {


//! Specialized type of a discrete trajectory for scalar types
/*!
 * \tparam SCALAR scalar data type
 */
template <class SCALAR = double, class TIME_SCALAR = double>
class ScalarTrajectory : public DiscreteTrajectory<SCALAR, Eigen::aligned_allocator<SCALAR>>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BASE = DiscreteTrajectory<SCALAR, Eigen::aligned_allocator<SCALAR>>;

    //! default constructor
    ScalarTrajectory() = default;

    //! resize constructor
    /*!
	 * initializes an array with a certain length and fills it with a default value
	 * @param n length of array
	 * @param value default value
	 */
    ScalarTrajectory(size_t n, const SCALAR& value = SCALAR()) : BASE(n, value){};

    //! copy constructor
    ScalarTrajectory(const ScalarTrajectory& other) : BASE(other){};

    //! constructor from std::vector
    ScalarTrajectory(const std::vector<SCALAR>& arg) : BASE()
    {
        for (size_t i = 0; i < arg.size(); i++)
            this->push_back(arg[i]);
    }

    //! destructor
    virtual ~ScalarTrajectory() = default;
    //! convert to an Eigen trajectory
    std::vector<Eigen::Matrix<SCALAR, 1, 1>, Eigen::aligned_allocator<Eigen::Matrix<SCALAR, 1, 1>>> toEigenTrajectory()
    {
        std::vector<Eigen::Matrix<SCALAR, 1, 1>, Eigen::aligned_allocator<Eigen::Matrix<SCALAR, 1, 1>>> eigenTraj;
        for (size_t i = 0; i < this->size(); i++)
        {
            Eigen::Matrix<SCALAR, 1, 1> newElement;
            newElement(0, 0) = (*this)[i];
            eigenTraj.push_back(newElement);
        }
        return eigenTraj;
    }

private:
};
}  // namespace core
}  // namespace ct
