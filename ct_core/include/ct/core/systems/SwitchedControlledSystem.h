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
template <typename MANIFOLD, size_t CONTROL_DIM, bool CONT_T>
class SwitchedControlledSystem : public ControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SCALAR = typename MANIFOLD::Scalar;

    using Base = ControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T>;
    using ControlledSystem_t = Base;
    using Time_t = typename Base::Time_t;

    using ModeSequence_t = PhaseSequence<std::size_t, Time_t>;

    typedef typename std::shared_ptr<SwitchedControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T>> Ptr;
    typedef typename std::shared_ptr<ControlledSystem_t> SystemPtr;
    typedef Switched<SystemPtr> SwitchedSystems;


    //! default constructor
    /*!
     * @param type system type
     */
    SwitchedControlledSystem(const SwitchedSystems& switchedSystems,
        const ModeSequence_t& modeSequence,
        const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL)
        : ControlledSystem_t(type), switchedSystems_(switchedSystems), modeSequence_(modeSequence){};

    //! constructor
    /*!
     *
     * @param controller controller
     * @param type system type
     */
    SwitchedControlledSystem(const SwitchedSystems& switchedSystems,
        const ModeSequence_t& modeSequence,
        std::shared_ptr<ct::core::Controller<MANIFOLD, CONTROL_DIM, CONT_T>> controller,
        const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL)
        : ControlledSystem_t(controller, type), switchedSystems_(switchedSystems), modeSequence_(modeSequence){};

    //! copy constructor
    SwitchedControlledSystem(const SwitchedControlledSystem& arg)
        : ControlledSystem_t(arg), modeSequence_(arg.modeSequence_)
    {
        switchedSystems_.clear();
        for (auto& subSystem : arg.switchedSystems_)
        {
            switchedSystems_.emplace_back(subSystem->clone());
        }
    };

    virtual ~SwitchedControlledSystem(){};

    virtual SwitchedControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T>* clone() const override
    {
        return new SwitchedControlledSystem(*this);
    };

    virtual void computeControlledDynamics(const MANIFOLD& state,
        const Time_t& tn,
        const ControlVector<CONTROL_DIM, SCALAR>& control,
        typename MANIFOLD::Tangent& dx) override
    {
        auto mode = modeSequence_.getPhaseFromTime(tn);
        switchedSystems_[mode]->computeControlledDynamics(state, tn, control, dx);
    };

protected:
    SwitchedSystems switchedSystems_;  //!< switched system container
    ModeSequence_t modeSequence_;      //!< the prespecified mode sequence
};
}
}
