/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

//! class for a general switched discrete linear system or linearized discrete system
/*!
 * Defines the interface for a switched discrete linear system
 *
 * \tparam STATE_DIM size of state vector
 * \tparam CONTROL_DIM size of input vector
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class SwitchedDiscreteLinearSystem : public DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename std::shared_ptr<DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>> LinearSystemPtr;
    typedef Switched<LinearSystemPtr> SwitchedLinearSystems;

    typedef DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR> Base;
    typedef typename Base::time_t time_t;

    typedef typename Base::state_vector_t state_vector_t;
    typedef typename Base::control_vector_t control_vector_t;

    typedef StateMatrix<STATE_DIM, SCALAR> state_matrix_t;                              //!< state Jacobian type
    typedef StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR> state_control_matrix_t;  //!< input Jacobian type

    //! default constructor
    /*!
     * @param type system type
     */
    SwitchedDiscreteLinearSystem(const SwitchedLinearSystems& switchedLinearSystems,
        const DiscreteModeSequence& discreteModeSequence,
        const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL)
        : DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>(type),
          switchedLinearSystems_(switchedLinearSystems),
          discreteModeSequence_(discreteModeSequence){};

    //! copy constructor
    SwitchedDiscreteLinearSystem(const SwitchedDiscreteLinearSystem& arg)
        : DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>(arg), discreteModeSequence_(arg.discreteModeSequence_)
    {
        switchedLinearSystems_.clear();
        for (auto& subSystem : arg.switchedLinearSystems_)
        {
            switchedLinearSystems_.emplace_back(subSystem->clone());
        }
    };

    //! destructor
    virtual ~SwitchedDiscreteLinearSystem(){};

    //! deep cloning
    virtual SwitchedDiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
    {
        return new SwitchedDiscreteLinearSystem(*this);
    };

    //! compute the system dynamics
    /*!
     * This computes the system dynamics
     * \f[
     *  x_{n+1} = A_{i}x_n + B_{i}u_n
     * \f]
     * @param state current state
     * @param n current time index
     * @param control control input
     * @param stateNext propagated state
     */
    virtual void propagateControlledDynamics(const state_vector_t& state,
        const time_t n,
        const control_vector_t& control,
        state_vector_t& stateNext) override
    {
        auto mode = discreteModeSequence_.getPhaseFromTime(n);
        switchedLinearSystems_[mode]->propagateControlledDynamics(state, n, control, stateNext);
    };

    //! retrieve discrete-time linear system matrices A and B for mode i, active at time n.
    /*!
     * This computes matrices A and B such that
     * \f[
     *  x_{n+1} = A_{i}x_n + B_{i}u_n
     * \f]
     *
     * Note that the inputs x_next and subSteps are potentially being ignored
     * for 'true' discrete system models but are relevant for sensitivity
     * calculations if the underlying system is continuous.
     *
     * @param x the state setpoint at n
     * @param u the control setpoint at n
     * @param n the time setpoint
     * @param x_next the state at n+1
     * @param subSteps number of substeps to use in sensitivity calculation
     * @param A the resulting linear system matrix A
     * @param B the resulting linear system matrix B
     */
    virtual void getAandB(const state_vector_t& x,
        const control_vector_t& u,
        const state_vector_t& x_next,
        const int n,
        size_t subSteps,
        state_matrix_t& A,
        state_control_matrix_t& B) override
    {
        auto mode = discreteModeSequence_.getPhaseFromTime(n);
        switchedLinearSystems_[mode]->getAandB(x, u, x_next, n, subSteps, A, B);
    };

    using DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>::getAandB;

protected:
    SwitchedLinearSystems switchedLinearSystems_;  //!< Switched linear system container
    DiscreteModeSequence discreteModeSequence_;    //!< the prespecified mode sequence
};

}  // namespace core
}  // namespace ct