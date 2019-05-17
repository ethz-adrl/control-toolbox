/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "DiscreteArray.h"

namespace ct {
namespace core {

//! An array of scalar data types
/*!
 * This class stores an array of scalar data types.
 */
template <class SCALAR = double>
class ScalarArray : public DiscreteArray<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::vector<Eigen::Matrix<SCALAR, 1, 1>, Eigen::aligned_allocator<Eigen::Matrix<SCALAR, 1, 1>>> EigenTraj;

    //! default constructor
    ScalarArray(){};

    //! resize constructor
    /*!
	 * initializes an array with a certain length and fills it with a default value
	 * @param n length of array
	 * @param value default value
	 */
    ScalarArray(size_t n, const SCALAR& value = SCALAR()) : DiscreteArray<SCALAR>(n, value){};

    //! copy constructor
    ScalarArray(const ScalarArray& other) : DiscreteArray<SCALAR>(other){};

    //! constructor from std::vector
    ScalarArray(const std::vector<SCALAR>& arg) : DiscreteArray<SCALAR>()
    {
        for (size_t i = 0; i < arg.size(); i++)
            this->push_back(arg[i]);
    }

    //! destructor
    virtual ~ScalarArray() {}
    void fromEigenTrajectory(const EigenTraj in)
    {
        for (auto el : in)
            this->push_back(el(0, 0));
    }

    //! convert to an Eigen trajectory
    EigenTraj toEigenTrajectory()
    {
        EigenTraj eigenTraj;
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
