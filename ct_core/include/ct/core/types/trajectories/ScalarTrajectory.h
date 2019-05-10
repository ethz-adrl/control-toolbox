/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "DiscreteTrajectoryBase.h"

namespace ct {
namespace core {


//! Specialized type of a discrete trajectory for scalar types
/*!
 * \tparam SCALAR scalar data type
 */
template <class SCALAR = double, class TIME_SCALAR = double>
class ScalarTrajectory : public DiscreteTrajectoryBase<SCALAR, Eigen::aligned_allocator<SCALAR>, TIME_SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! default constructor
    ScalarTrajectory(){};

    //! resize constructor
    /*!
	 * initializes an array with a certain length and fills it with a default value
	 * @param n length of array
	 * @param value default value
	 */
    ScalarTrajectory(size_t n, const SCALAR& value = SCALAR()) : DiscreteTrajectoryBase<SCALAR>(n, value){};

    //! copy constructor
    ScalarTrajectory(const ScalarTrajectory& other) : DiscreteTrajectoryBase<SCALAR>(other){};

    //! constructor from std::vector
    ScalarTrajectory(const std::vector<SCALAR>& arg) : DiscreteTrajectoryBase<SCALAR>()
    {
        for (size_t i = 0; i < arg.size(); i++)
            this->push_back(arg[i]);
    }

    //! destructor
    virtual ~ScalarTrajectory() {}
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
}
}
