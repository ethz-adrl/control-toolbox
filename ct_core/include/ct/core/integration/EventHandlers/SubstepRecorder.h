/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/integration/EventHandler.h>
#include <ct/core/systems/ControlledSystem.h>

namespace ct {
namespace core {

//! Event handler to record substeps
/*!
 * This event handler records subintegration steps
 *
 * @tparam STATE_DIM size of the state vector
 */
template <typename MANIFOLD, size_t CONTROL_DIM>
class SubstepRecorder : public EventHandler<MANIFOLD>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SCALAR = typename MANIFOLD::Scalar;

    using ManifoldArray_t = DiscreteArray<MANIFOLD>;
    using ControlVectorArray_t = ControlVectorArray<CONTROL_DIM, SCALAR>;
    using TimeArray_t = tpl::TimeArray<SCALAR>;
    using ControlledSystem_t = ControlledSystem<MANIFOLD, CONTROL_DIM, CONTINUOUS_TIME>;

    SubstepRecorder(std::shared_ptr<ControlledSystem_t> system = nullptr,
        bool activated = true,
        std::shared_ptr<ManifoldArray_t> states = std::shared_ptr<ManifoldArray_t>(new ManifoldArray_t()),
        std::shared_ptr<ControlVectorArray_t> controls = std::shared_ptr<ControlVectorArray_t>(
            new ControlVectorArray_t()),
        std::shared_ptr<TimeArray_t> times = std::shared_ptr<TimeArray_t>(new TimeArray_t()));

    virtual ~SubstepRecorder();

    virtual bool callOnSubsteps() override;

    void setControlledSystem(const std::shared_ptr<ControlledSystem_t>& system);

    //! checks the kill flag
    virtual bool checkEvent(const MANIFOLD& state, const SCALAR& t) override;

    //! records the state
    virtual void handleEvent(const MANIFOLD& state, const SCALAR& t) override;

    void setEnable(bool activated);

    //! resets kill flag to false
    virtual void reset() override;

    const std::shared_ptr<ManifoldArray_t>& getSubstates() const;

    const std::shared_ptr<ControlVectorArray<CONTROL_DIM, SCALAR>>& getSubcontrols() const;

private:
    bool activated_;

    std::shared_ptr<ControlledSystem_t> system_;

    //!< container for logging the state
    std::shared_ptr<ManifoldArray_t> states_;

    //!< container for logging the control
    std::shared_ptr<ControlVectorArray_t> controls_;

    //!< container for logging the time
    std::shared_ptr<TimeArray_t> times_;
};
}  // namespace core
}  // namespace ct
