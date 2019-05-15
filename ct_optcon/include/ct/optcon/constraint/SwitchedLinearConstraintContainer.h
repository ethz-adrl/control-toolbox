/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/core.h>
#include "LinearConstraintContainer.h"

namespace ct {
namespace optcon {


/**
 * @ingroup    Constraint
 *
 * @brief      A container for switching linear constraint containers
 *
 * * The LinearConstraintBase Class is the base class for defining the
 *   non-linear optimization constraints. This class switches between several of them based on a predefined sequence.
 *
 * @tparam     STATE_DIM  Dimension of the state vector
 * @tparam     CONTROL_DIM  Dimension of the control input vector
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class SwitchedLinearConstraintContainer : public LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef core::ControlVector<CONTROL_DIM, SCALAR> input_vector_t;

    typedef SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>*
        SwitchedLinearConstraintContainer_Raw_Ptr_t;
    typedef std::shared_ptr<LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>> LinearConstraintContainer_Ptr_t;
    typedef core::Switched<LinearConstraintContainer_Ptr_t> SwitchedLinearConstraintContainers;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

    typedef core::PhaseSequence<std::size_t, SCALAR> ModeSequence_t;

    /**
     * @brief      Default constructor
     */
    SwitchedLinearConstraintContainer(const SwitchedLinearConstraintContainers& switchedLinearConstraintContainers,
        const ModeSequence_t& continuousModeSequence);


    /**
     * @brief      Copy constructor
     *
     * @param[in]  arg   The object to be copied
     */
    SwitchedLinearConstraintContainer(const SwitchedLinearConstraintContainer& arg);
    /**
     * @brief      Destructor
     *
     */
    virtual ~SwitchedLinearConstraintContainer();

    /**
     * Clones the linear constraint class
     * @return pointer to the clone
     */
    virtual SwitchedLinearConstraintContainer_Raw_Ptr_t clone() const override;

    virtual VectorXs evaluateIntermediate() override;

    virtual VectorXs evaluateTerminal() override;

    virtual size_t getIntermediateConstraintsCount() override;

    virtual size_t getTerminalConstraintsCount() override;

    virtual VectorXs jacobianStateSparseIntermediate() override;

    virtual MatrixXs jacobianStateIntermediate() override;

    virtual VectorXs jacobianStateSparseTerminal() override;

    virtual MatrixXs jacobianStateTerminal() override;

    virtual VectorXs jacobianInputSparseIntermediate() override;

    virtual MatrixXs jacobianInputIntermediate() override;

    virtual VectorXs jacobianInputSparseTerminal() override;

    virtual MatrixXs jacobianInputTerminal() override;

    virtual void sparsityPatternStateIntermediate(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override;

    virtual void sparsityPatternStateTerminal(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override;

    virtual void sparsityPatternInputIntermediate(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override;

    virtual void sparsityPatternInputTerminal(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override;

    virtual size_t getJacobianStateNonZeroCountIntermediate() override;

    virtual size_t getJacobianStateNonZeroCountTerminal() override;

    virtual size_t getJacobianInputNonZeroCountIntermediate() override;

    virtual size_t getJacobianInputNonZeroCountTerminal() override;

    virtual bool initializeIntermediate() override;

    virtual bool initializeTerminal() override;

private:
    virtual void update() override;

    SwitchedLinearConstraintContainers switchedLinearConstraintContainers_;
    ModeSequence_t continuousModeSequence_;
    LinearConstraintContainer_Ptr_t activeLinearConstraintContainer_;
    LinearConstraintContainer_Ptr_t terminalLinearConstraintContainer_;
};

}  // namespace optcon
}  // namespace ct
