/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/types/StateVector.h>
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
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class SwitchedLinearSystem : public LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename std::shared_ptr<LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>> LinearSystemPtr;
    typedef Switched<LinearSystemPtr> SwitchedLinearSystems;

    typedef ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR> Base;
    typedef typename Base::time_t time_t;

    typedef StateVector<STATE_DIM, SCALAR> state_vector_t;        //!< state vector type
    typedef ControlVector<CONTROL_DIM, SCALAR> control_vector_t;  //!< input vector type

    typedef StateMatrix<STATE_DIM, SCALAR> state_matrix_t;                              //!< state Jacobian type
    typedef StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR> state_control_matrix_t;  //!< input Jacobian type

    //! default constructor
    /*!
     * @param type system type
     */
    SwitchedLinearSystem(const SwitchedLinearSystems& switchedLinearSystems,
        const ContinuousModeSequence& continuousModeSequence,
        const ct::core::SYSTEM_TYPE& type = ct::core::SYSTEM_TYPE::GENERAL)
        : LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>(type),
          switchedLinearSystems_(switchedLinearSystems),
          continuousModeSequence_(continuousModeSequence)
    {
    }

    //! copy constructor
    SwitchedLinearSystem(const SwitchedLinearSystem& arg)
        : LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>(arg), continuousModeSequence_(arg.continuousModeSequence_)
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
    virtual SwitchedLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
    {
        return new SwitchedLinearSystem(*this);
    };

    using LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>::computeControlledDynamics;

    virtual const state_matrix_t& getDerivativeState(const state_vector_t& x,
        const control_vector_t& u,
        const time_t t = time_t(0.0)) override
    {
        auto mode = continuousModeSequence_.getPhaseFromTime(t);
        return switchedLinearSystems_[mode]->getDerivativeState(x, u, t);
    };

    virtual const state_control_matrix_t& getDerivativeControl(const state_vector_t& x,
        const control_vector_t& u,
        const time_t t = time_t(0.0)) override
    {
        auto mode = continuousModeSequence_.getPhaseFromTime(t);
        return switchedLinearSystems_[mode]->getDerivativeControl(x, u, t);
    };

private:
    SwitchedLinearSystems switchedLinearSystems_;    //!< Switched linear system container
    ContinuousModeSequence continuousModeSequence_;  //!< the prespecified mode sequence
};
}
}
