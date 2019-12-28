/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/core.h>

#include "LinearConstraintContainer.h"
#include "term/ConstraintBase.h"


namespace ct {
namespace optcon {

/**
 * @ingroup    Constraint
 *
 * @brief      Contains all the constraints using analytically calculated
 *             jacobians
 *
 * @tparam     STATE_DIM  The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ConstraintContainerAnalytical : public LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef core::ControlVector<CONTROL_DIM, SCALAR> input_vector_t;

    typedef ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>* ConstraintContainerAnalytical_Raw_Ptr_t;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

    ConstraintContainerAnalytical();

    /**
	 * @brief      Constructor using state, control and time
	 *
	 * @param      x     state vector
	 * @param      u     control vector
	 * @param      t     time
	 */
    ConstraintContainerAnalytical(const state_vector_t& x, const input_vector_t& u, const SCALAR& t = 0.0);

    /**
	 * @brief      Copy constructor
	 *
	 * @param      arg   constraint class to copy
	 */
    ConstraintContainerAnalytical(const ConstraintContainerAnalytical& arg);

    /**
	 * @brief      Deep-cloning of Constraint
	 *
	 * @return     Copy of this object.
	 */
    virtual ConstraintContainerAnalytical_Raw_Ptr_t clone() const override;

    /**
	 * @brief      Destructor
	 */
    virtual ~ConstraintContainerAnalytical();

    /**
	 * @brief      Adds an intermedaite constraint.
	 *
	 * @param[in]  constraint  The constraint to be added
	 * @param[in]  verbose     Flag indicating whether verbosity is on or off
	 */
    void addIntermediateConstraint(std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>> constraint,
        bool verbose);

    /**
	 * @brief      Adds a terminal constraint.
	 *
	 * @param[in]  constraint  The constraint to be added
	 * @param[in]  verbose     Flag indicating whether verbosity is on or off
	 */
    void addTerminalConstraint(std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>> constraint,
        bool verbose);

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

    /**
	 * @brief      Checks whether the intermediate constraints are initialized.
	 *             Throws a runtime error if not.
	 */
    void checkIntermediateConstraints();

    /**
	 * @brief      Checks whether the terminal constraints are initialized.
	 *             Throws a runtime error if not.
	 */
    void checkTerminalConstraints();


    std::vector<std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>>> constraintsIntermediate_;
    std::vector<std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>>> constraintsTerminal_;

    VectorXs evalIntermediate_;
    VectorXs evalJacSparseStateIntermediate_;
    VectorXs evalJacSparseInputIntermediate_;
    MatrixXs evalJacDenseStateIntermediate_;
    MatrixXs evalJacDenseInputIntermediate_;

    VectorXs evalTerminal_;
    VectorXs evalJacSparseStateTerminal_;
    VectorXs evalJacSparseInputTerminal_;
    MatrixXs evalJacDenseStateTerminal_;
    MatrixXs evalJacDenseInputTerminal_;
};


}  // namespace optcon
}  // namespace ct
