/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "FilterSettings.h"

namespace ct {
namespace optcon {

/*!
 * \ingroup Filter
 *
 * \brief Estimator base provides an estimator interface which simply stores the state estimate and provides getters
 *        and setters.
 *
 * @tparam STATE_DIM
 */
template <size_t STATE_DIM, typename SCALAR = double>
class EstimatorBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using state_vector_t = ct::core::StateVector<STATE_DIM, SCALAR>;

    //! Constructor.
    EstimatorBase(const ct::core::StateVector<STATE_DIM, SCALAR>& x0 = ct::core::StateVector<STATE_DIM, SCALAR>::Zero())
        : x_est_(x0)
    {
    }
    //! Copy constructor.
    EstimatorBase(const EstimatorBase& arg) : x_est_(arg.x_est_) {}
    //! Estimate getter.
    const ct::core::StateVector<STATE_DIM, SCALAR>& getEstimate() const { return x_est_; }
    //! Estimate setter.
    void setEstimate(const ct::core::StateVector<STATE_DIM, SCALAR>& x) { x_est_ = x; }
protected:
    ct::core::StateVector<STATE_DIM, SCALAR> x_est_;  //! State estimate.
};

}  // optcon
}  // ct
