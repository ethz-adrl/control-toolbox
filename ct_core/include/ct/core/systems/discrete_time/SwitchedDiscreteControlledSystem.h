/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "DiscreteControlledSystem.h"
#include <ct/core/control/discrete_time/DiscreteController.h>
#include <ct/core/switching/Switching.h>

namespace ct {
namespace core {

//! A general, switched non-linear discrete dynamic system with a control input
/*!
 * This describes a general, switched non-linear discrete dynamic system of the following form
 *
 * \f[
 *  x_{n+1} = f_{i}(x_n,u_n,n)
 * \f]
 *
 * where \f$ x_{n} \f$ is the state, \f$ u_{n} \f$ the control input and \f$ n \f$ the time index.
 * \f$ f_{i} \f$ refers to the dynamics in a specific mode. Modes are prespecified as a function of time.
 *
 *
 * For implementing your own SwitchedDiscreteControlledSystem, provide a vector of Discrete systems
 * and a prespecified mode sequence.
 *
 * We generally assume that the Controller is a state and time index dependent function \f$ u_n = g(x_n,n) \f$
 * which allows any ControlledSystem to be re-written as a System of the form
 *
 * \f[
 *  x_{n+1} = f(x_n,u_n(x_n,n),n) = g(x_n,n)
 * \f]
 *
 * which can be forward propagated directly.
 *
 * @tparam STATE_DIM dimension of state vector
 * @tparam CONTROL_DIM dimension of input vector
 * @tparam SCALAR scalar type
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class SwitchedDiscreteControlledSystem : public DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename std::shared_ptr<SwitchedDiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> Ptr;
    typedef typename std::shared_ptr<DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> SystemPtr;
    typedef Switched<SystemPtr> SwitchedSystems;

    typedef DiscreteSystem<STATE_DIM, CONTROL_DIM, SCALAR> Base;

    typedef typename Base::state_vector_t state_vector_t;
    typedef typename Base::control_vector_t control_vector_t;
    typedef typename Base::time_t time_t;

    //! default constructor
    /*!
     * @param type system type
     */
    SwitchedDiscreteControlledSystem(const SwitchedSystems& switchedSystems,
        const DiscreteModeSequence& discreteModeSequence,
        const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL)
        : DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>(type),
          switchedSystems_(switchedSystems),
          discreteModeSequence_(discreteModeSequence){};

    //! constructor
    /*!
     *
     * @param controller controller
     * @param type system type
     */
    SwitchedDiscreteControlledSystem(const SwitchedSystems& switchedSystems,
        const DiscreteModeSequence& discreteModeSequence,
        std::shared_ptr<DiscreteController<STATE_DIM, CONTROL_DIM, SCALAR>> controller,
        const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL)
        : DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>(controller, type),
          switchedSystems_(switchedSystems),
          discreteModeSequence_(discreteModeSequence){};

    //! copy constructor
    SwitchedDiscreteControlledSystem(const SwitchedDiscreteControlledSystem& arg)
        : DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>(arg),
          discreteModeSequence_(arg.discreteModeSequence_)
    {
        switchedSystems_.clear();
        for (auto& subSystem : arg.switchedSystems_)
        {
            switchedSystems_.emplace_back(subSystem->clone());
        }
    };

    //! destructor
    virtual ~SwitchedDiscreteControlledSystem(){};

    //! deep copy
    virtual SwitchedDiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
    {
        return new SwitchedDiscreteControlledSystem(*this);
    };

    //! propagates the controlled system dynamics forward by one step
    /*!
     * evaluates \f$ x_{n+1} = f(x_n, u_n, n) \f$ at a given state, control, index, and mode
     * @param state start state to propagate from
     * @param control the control input to apply. This is a constant control input applied to the continuous-time dynamics
     * @param n time index to propagate the dynamics at
     * @param stateNext the resulting propagated state
     */
    virtual void propagateControlledDynamics(const state_vector_t& state,
        const time_t n,
        const control_vector_t& control,
        state_vector_t& stateNext) override
    {
        auto mode = discreteModeSequence_.getPhaseFromTime(n);
        switchedSystems_[mode]->propagateControlledDynamics(state, n, control, stateNext);
    };

protected:
    SwitchedSystems switchedSystems_;            //!< switched system container
    DiscreteModeSequence discreteModeSequence_;  //!< the prespecified mode sequence
};
}  // namespace core
}  // namespace ct
