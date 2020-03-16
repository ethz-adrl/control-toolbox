/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "ControlledSystem.h"
#include <ct/core/switching/Switching.h>

namespace ct {
namespace core {

//! A general, switched non-linear dynamic system with a control input
/*!
 * This describes a general, switched non-linear dynamic system described by an Ordinary Differential Equation (ODE)
 * of the following form
 *
 * \f[
 *  \dot{x} = f_{i}(x,u,t)
 * \f]
 *
 * where \f$ x(t) \f$ is the state, \f$ u(t) \f$ the control input and \f$ t \f$ the time.
 * \f$ f_{i} \f$ refers to the dynamics in a specific mode. Modes are prespecified as a function of time.
 *
 * For implementing your own SwitchedControlledSystem, provide a vector of controlled systems
 * and a prespecified mode sequence.
 *
 * We generally assume that the Controller is a state and time dependent function \f$ u = g(x,t) \f$
 * which allows any ControlledSystem to be re-written as a System of the form
 *
 * \f[
 *  \dot{x} = f(x(t),u(x,t),t) = g(x,t)
 * \f]
 *
 * which can be forward propagated in time with an Integrator.
 *
 * @tparam STATE_DIM dimension of state vector
 * @tparam CONTROL_DIM dimension of input vector
 * @tparam SCALAR scalar type
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class SwitchedControlledSystem : public ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename std::shared_ptr<SwitchedControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> Ptr;
    typedef typename std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> SystemPtr;
    typedef Switched<SystemPtr> SwitchedSystems;

    typedef System<STATE_DIM, SCALAR> Base;
    typedef typename Base::time_t time_t;


    //! default constructor
    /*!
     * @param type system type
     */
    SwitchedControlledSystem(const SwitchedSystems& switchedSystems,
        const ContinuousModeSequence& continuousModeSequence,
        const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL)
        : ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>(type),
          switchedSystems_(switchedSystems),
          continuousModeSequence_(continuousModeSequence){};

    //! constructor
    /*!
     *
     * @param controller controller
     * @param type system type
     */
    SwitchedControlledSystem(const SwitchedSystems& switchedSystems,
        const ContinuousModeSequence& continuousModeSequence,
        std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller,
        const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL)
        : ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>(controller, type),
          switchedSystems_(switchedSystems),
          continuousModeSequence_(continuousModeSequence){};

    //! copy constructor
    SwitchedControlledSystem(const SwitchedControlledSystem& arg)
        : ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>(arg), continuousModeSequence_(arg.continuousModeSequence_)
    {
        switchedSystems_.clear();
        for (auto& subSystem : arg.switchedSystems_)
        {
            switchedSystems_.emplace_back(subSystem->clone());
        }
    };

    //! destructor
    virtual ~SwitchedControlledSystem(){};

    //! deep copy
    virtual SwitchedControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
    {
        return new SwitchedControlledSystem(*this);
    };

    virtual void computeControlledDynamics(const StateVector<STATE_DIM, SCALAR>& state,
        const time_t& t,
        const ControlVector<CONTROL_DIM, SCALAR>& control,
        StateVector<STATE_DIM, SCALAR>& derivative) override
    {
        auto mode = continuousModeSequence_.getPhaseFromTime(t);
        switchedSystems_[mode]->computeControlledDynamics(state, t, control, derivative);
    };

protected:
    SwitchedSystems switchedSystems_;                //!< switched system container
    ContinuousModeSequence continuousModeSequence_;  //!< the prespecified mode sequence
};
}
}
