/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "FilterBase.h"
#include "FilterSettings.h"
#include "EstimatorBase.h"
#include "LTIMeasurementModel.h"

namespace ct {
namespace optcon {

template <size_t OUTPUT_DIM, size_t STATE_DIM, size_t CONTROL_DIM, class ESTIMATOR, typename SCALAR = double>
class StateObserver : public FilterBase<OUTPUT_DIM, STATE_DIM, SCALAR>
{
public:
    static_assert(STATE_DIM == ESTIMATOR::STATE_D, "Observer and estimator dimensions have to be the same!");

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Base = FilterBase<OUTPUT_DIM, STATE_DIM, SCALAR>;
    using typename Base::state_vector_t;
    using typename Base::output_vector_t;
    using typename Base::Time_t;
    using state_matrix_t        = ct::core::StateMatrix<STATE_DIM, SCALAR>;
    using output_matrix_t       = ct::core::OutputMatrix<OUTPUT_DIM, SCALAR>;
    using output_state_matrix_t = ct::core::OutputStateMatrix<OUTPUT_DIM, STATE_DIM, SCALAR>;

    StateObserver(std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> system,
        const ct::core::SensitivityApproximation<STATE_DIM, CONTROL_DIM, STATE_DIM / 2, STATE_DIM / 2, SCALAR>&
            sensApprox,
        double dt,
        const output_state_matrix_t& C,
        const ESTIMATOR& estimator,
        const state_matrix_t& Q,
        const output_matrix_t& R)
        : estimator_(estimator), f_(system, sensApprox, dt), h_(C), Q_(Q), R_(R)
    {
    }

    StateObserver(std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> system,
        const ct::core::SensitivityApproximation<STATE_DIM, CONTROL_DIM, STATE_DIM / 2, STATE_DIM / 2, SCALAR>&
            sensApprox,
        const ESTIMATOR& estimator,
        const StateObserverSettings<OUTPUT_DIM, STATE_DIM, SCALAR>& so_settings)
        : f_(system, sensApprox, so_settings.dt),
          h_(so_settings.C),
          estimator_(estimator),
          Q_(so_settings.Q),
          R_(so_settings.R)
    {
    }

    virtual ~StateObserver() {}
    state_vector_t filter(const output_vector_t& y, const Time_t& t) override
    {
        predict(t);
        return update(y, t);
    }
    virtual state_vector_t predict(const Time_t& t = 0)
    {
        return estimator_.template predict<CONTROL_DIM>(
            f_, ct::core::ControlVector<CONTROL_DIM, SCALAR>::Zero(), Q_, t);
    }

    virtual state_vector_t update(const output_vector_t& y, const Time_t& t = 0)
    {
        return estimator_.template update<OUTPUT_DIM>(y, h_, R_, t);
    }

protected:
    ESTIMATOR estimator_;
    CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR> f_;
    LTIMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR> h_;
    state_matrix_t Q_;
    output_matrix_t R_;
};

}  // optcon
}  // ct
