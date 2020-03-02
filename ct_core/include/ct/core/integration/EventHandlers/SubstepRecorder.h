/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/integration/EventHandler.h>
#include <ct/core/systems/continuous_time/ControlledSystem.h>

namespace ct {
namespace core {

//! Event handler to record substeps
/*!
 * This event handler records subintegration steps
 *
 * @tparam STATE_DIM size of the state vector
 */
template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR = double>
class SubstepRecorder : public EventHandler<MANIFOLD, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using ManifoldArray_t = ct::core::DiscreteArray<MANIFOLD, SCALAR>;
    using ControlVectorArray_t = ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>;
    using TimeArray_t = ct::core::tpl::TimeArray<SCALAR>;

    SubstepRecorder(std::shared_ptr<ControlledSystem<MANIFOLD, CONTROL_DIM, SCALAR>> system = nullptr,
        bool activated = true,
        std::shared_ptr<ManifoldArray_t> states = std::shared_ptr<ManifoldArray_t>(new ManifoldArray_t()),
        std::shared_ptr<ControlVectorArray_t> controls = std::shared_ptr<ControlVectorArray_t>(
            new ControlVectorArray_t()),
        std::shared_ptr<TimeArray_t> times = std::shared_ptr<TimeArray_t>(new TimeArray_t()));

    virtual ~SubstepRecorder();

    virtual bool callOnSubsteps() override;

    void setControlledSystem(const std::shared_ptr<ControlledSystem<MANIFOLD, CONTROL_DIM, SCALAR>>& system);

    //! checks the kill flag
    virtual bool checkEvent(const MANIFOLD& state, const SCALAR& t) override;

    //! records the state
    virtual void handleEvent(const MANIFOLD& state, const SCALAR& t) override;

    void setEnable(bool activated);

    //! resets kill flag to false
    virtual void reset() override;

    const std::shared_ptr<ManifoldArray_t>& getSubstates() const;

    const std::shared_ptr<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>>& getSubcontrols() const;

private:
    bool activated_;

    std::shared_ptr<ct::core::ControlledSystem<MANIFOLD, CONTROL_DIM, SCALAR>> system_;

    //!< container for logging the state
    std::shared_ptr<ManifoldArray_t> states_;

    //!< container for logging the control
    std::shared_ptr<ControlVectorArray_t> controls_;

    //!< container for logging the time
    std::shared_ptr<TimeArray_t> times_;
};
}  // namespace core
}  // namespace ct
