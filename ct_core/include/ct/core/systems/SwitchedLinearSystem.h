/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/types/ControlVector.h>

namespace ct {
namespace core {

//! interface class for a general switched linear system or linearized system
/*!
 * Defines the interface for a switched linear system
 *
 * \tparam STATE_DIM size of state vector
 * \tparam CONTROL_DIM size of input vector
 */
template <typename MANIFOLD, size_t CONTROL_DIM, bool CONT_T>
class SwitchedLinearSystem : public LinearSystem<MANIFOLD, CONTROL_DIM, CONT_T>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_DIM = MANIFOLD::TangentDim;

    using SCALAR = typename MANIFOLD::Scalar;

    using Base = LinearSystem<MANIFOLD, CONTROL_DIM, CONT_T>;
    typedef typename Base::Time_t Time_t;

    using ModeSequence_t = PhaseSequence<std::size_t, Time_t>;

    typedef typename std::shared_ptr<Base> LinearSystemPtr;
    typedef Switched<LinearSystemPtr> SwitchedLinearSystems;

    typedef ControlVector<CONTROL_DIM, SCALAR> control_vector_t;       //!< input vector type
    typedef StateMatrix<MANIFOLD::TangentDim, SCALAR> state_matrix_t;  //!< state Jacobian type
    typedef StateControlMatrix<MANIFOLD::TangentDim, CONTROL_DIM, SCALAR>
        state_control_matrix_t;  //!< input Jacobian type

    //! default constructor
    /*!
     * @param type system type
     */
    SwitchedLinearSystem(const SwitchedLinearSystems& switchedLinearSystems,
        const ModeSequence_t& modeSequence,
        const ct::core::SYSTEM_TYPE& type = ct::core::SYSTEM_TYPE::GENERAL)
        : Base(type), switchedLinearSystems_(switchedLinearSystems), modeSequence_(modeSequence)
    {
    }

    //! copy constructor
    SwitchedLinearSystem(const SwitchedLinearSystem& arg) : Base(arg), modeSequence_(arg.modeSequence_)
    {
        switchedLinearSystems_.clear();
        for (auto& subSystem : arg.switchedLinearSystems_)
        {
            switchedLinearSystems_.emplace_back(subSystem->clone());
        }
    }

    //! destructor
    virtual ~SwitchedLinearSystem(){};

    //! deep cloning
    virtual SwitchedLinearSystem<MANIFOLD, CONTROL_DIM, CONT_T>* clone() const override
    {
        return new SwitchedLinearSystem(*this);
    };


    virtual const state_matrix_t& getDerivativeState(const MANIFOLD& x,
        const control_vector_t& u,
        const Time_t t = Time_t(0.0)) override
    {
        auto mode = modeSequence_.getPhaseFromTime(t);
        return switchedLinearSystems_[mode]->getDerivativeState(x, u, t);
    };

    virtual const state_control_matrix_t& getDerivativeControl(const MANIFOLD& x,
        const control_vector_t& u,
        const Time_t t = Time_t(0.0)) override
    {
        auto mode = modeSequence_.getPhaseFromTime(t);
        return switchedLinearSystems_[mode]->getDerivativeControl(x, u, t);
    };

private:
    SwitchedLinearSystems switchedLinearSystems_;  //!< Switched linear system container
    ModeSequence_t modeSequence_;                  //!< the prespecified mode sequence
};
}
}
