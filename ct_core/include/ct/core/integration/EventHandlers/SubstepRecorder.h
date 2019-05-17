/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/integration/EventHandler.h>

namespace ct {
namespace core {

//! Event handler to record substeps
/*!
 * This event handler records subintegration steps
 *
 * @tparam STATE_DIM size of the state vector
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class SubstepRecorder : public EventHandler<STATE_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef StateVector<STATE_DIM, SCALAR> state_t;

    SubstepRecorder(std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> system = nullptr,
        bool activated = true,
        std::shared_ptr<ct::core::StateVectorArray<STATE_DIM, SCALAR>> states =
            std::shared_ptr<ct::core::StateVectorArray<STATE_DIM, SCALAR>>(
                new ct::core::StateVectorArray<STATE_DIM, SCALAR>),
        std::shared_ptr<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>> controls =
            std::shared_ptr<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>>(
                new ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>),
        std::shared_ptr<ct::core::tpl::TimeArray<SCALAR>> times = std::shared_ptr<ct::core::tpl::TimeArray<SCALAR>>(
            new ct::core::tpl::TimeArray<SCALAR>))
        : activated_(activated), system_(system), states_(states), controls_(controls), times_(times)
    {
    }

    //! default destructor
    virtual ~SubstepRecorder() {}
    virtual bool callOnSubsteps() override { return true; }
    void setControlledSystem(const std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& system)
    {
        system_ = system;
    }

    //! checks the kill flag
    virtual bool checkEvent(const state_t& state, const SCALAR& t) override { return activated_; }
    //! records the state
    virtual void handleEvent(const state_t& state, const SCALAR& t) override
    {
        states_->push_back(state);
        controls_->push_back(system_->getLastControlAction());
        times_->push_back(t);
    }

    void setEnable(bool activated) { activated_ = activated; }
    //! resets kill flag to false
    virtual void reset() override
    {
        states_ = std::shared_ptr<ct::core::StateVectorArray<STATE_DIM, SCALAR>>(
            new ct::core::StateVectorArray<STATE_DIM, SCALAR>);
        controls_ = std::shared_ptr<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>>(
            new ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>);
        times_ = std::shared_ptr<ct::core::tpl::TimeArray<SCALAR>>(new ct::core::tpl::TimeArray<SCALAR>);
    };

    const std::shared_ptr<ct::core::StateVectorArray<STATE_DIM, SCALAR>>& getSubstates() const { return states_; }
    const std::shared_ptr<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>>& getSubcontrols() const
    {
        return controls_;
    }

private:
    bool activated_;

    std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> system_;

    std::shared_ptr<ct::core::StateVectorArray<STATE_DIM, SCALAR>> states_;  //!< container for logging the state
    std::shared_ptr<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>>
        controls_;                                             //!< container for logging the control
    std::shared_ptr<ct::core::tpl::TimeArray<SCALAR>> times_;  //!< container for logging the time
};
}
}
