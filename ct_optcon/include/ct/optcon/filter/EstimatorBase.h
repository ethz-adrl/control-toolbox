/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "FilterSettings.h"
#include "SystemModelBase.h"
#include "LinearMeasurementModel.h"

namespace ct {
namespace optcon {

/*!
 * \ingroup Filter
 *
 * \brief Estimator base
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, size_t OUTPUT_DIM, typename SCALAR = double>
class EstimatorBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using control_vector_t = ct::core::ControlVector<CONTROL_DIM, SCALAR>;
    using state_vector_t = ct::core::StateVector<STATE_DIM, SCALAR>;
    using state_matrix_t = ct::core::StateMatrix<STATE_DIM, SCALAR>;
    using output_vector_t = ct::core::OutputVector<OUTPUT_DIM, SCALAR>;
    using output_matrix_t = ct::core::OutputMatrix<OUTPUT_DIM, SCALAR>;

    //! Constructor.
    EstimatorBase(std::shared_ptr<SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>> f,
        std::shared_ptr<LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>> h,
        const state_vector_t& x0 = state_vector_t::Zero())
        : f_(f), h_(h), x_est_(x0)
    {
    }

    //! Copy constructor.
    EstimatorBase(const EstimatorBase& arg) : f_(arg.f_->clone()), h_(arg.h_->clone()), x_est_(arg.x_est_) {}
    virtual ~EstimatorBase() = default;

    //! Estimator predict method.
    virtual const state_vector_t& predict(const control_vector_t& u,
        const ct::core::Time& dt,
        const ct::core::Time& t) = 0;

    //! Estimator update method.
    virtual const state_vector_t& update(const output_vector_t& y,
        const ct::core::Time& dt,
        const ct::core::Time& t) = 0;

    //! update the system model
    void setSystemModel(std::shared_ptr<SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>> f) { f_ = f; }
    //! update the measurement model
    void setMeasurementModel(std::shared_ptr<LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>> h) { h_ = h; }
    //! Estimate getter.
    const state_vector_t& getEstimate() const { return x_est_; }
    //! Estimate setter.
    void setEstimate(const state_vector_t& x) { x_est_ = x; }
protected:
    //! System model for propagating the system.
    std::shared_ptr<SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>> f_;

    //! Observation model used to calculate the output error.
    std::shared_ptr<LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>> h_;

    //! State estimate.
    state_vector_t x_est_;
};

}  // namespace optcon
}  // namespace ct
