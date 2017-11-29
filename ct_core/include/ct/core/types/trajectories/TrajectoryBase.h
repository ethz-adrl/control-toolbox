/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/
#ifndef CT_TRAJECTORYBASE_H_
#define CT_TRAJECTORYBASE_H_

#include "../Time.h"

namespace ct {
namespace core {

//! Basic interface class for a trajectory
/*!
 * This defines a general interface for a trajectory, independent of its
 * implementation. Possible implementations are a DiscreteTrajectoryBase or
 * also splines, basis functions etc.
 *
 * \tparam T data type
 */
template <class T, typename SCALAR>
class TrajectoryBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! default constructor
    TrajectoryBase() {}
    //! destructor
    virtual ~TrajectoryBase(){};

    //! evaluate the trajectory at a certain time
    virtual T eval(const SCALAR& time) = 0;
};

} /* namespace core */
} /* namespace ct */

#endif  // CT_TRAJECTORYBASE_H_
