/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#ifdef CPPADCG

#include "LinearConstraintContainer.h"
#include "term/ConstraintBase.h"

namespace ct {
namespace optcon {

/**
 * @ingroup    Constraint
 *
 * @brief      Contains all the constraints using with AD generated jacobians
 *
 * @tparam     STATE_DIM  { description }
 * @tparam     CONTROL_DIM  { description }
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ConstraintContainerAD : public LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef core::DerivativesCppadJIT<STATE_DIM + CONTROL_DIM, -1> JacCG;
    typedef typename JacCG::CG_SCALAR CGScalar;

    typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef core::ControlVector<CONTROL_DIM, SCALAR> input_vector_t;

    typedef ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>* ConstraintContainerAD_Raw_Ptr_t;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;


    /**
	 * @brief      Basic constructor
	 */
    ConstraintContainerAD();

    /**
	 * \brief Constructor using state, control and time
	 * @param x state vector
	 * @param u control vector
	 * @param t time
	 */
    ConstraintContainerAD(const state_vector_t& x, const input_vector_t& u, const SCALAR& t = 0.0);


    /**
	 * \brief Copy constructor
	 * @param arg constraint class to copy
	 */
    ConstraintContainerAD(const ConstraintContainerAD& arg);

    /**
	 * Deep-cloning of Constraint
	 *
	 * @return     base pointer to the clone
	 */
    virtual ConstraintContainerAD_Raw_Ptr_t clone() const override;

    /**
	 * @brief      Destructor
	 */
    virtual ~ConstraintContainerAD();

    /**
	 * @brief      Adds an intermediate constraint.
	 *
	 * @param[in]  constraint  The constraint
	 * @param[in]  verbose     Flag indicating whether verbosity is on or off
	 */
    void addIntermediateConstraint(std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>> constraint,
        bool verbose);

    /**
	 * @brief      Adds a terminal constraint.
	 *
	 * @param[in]  constraint  The constraint
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
    // cache the matrix here, no need to call it at every eval matrix
    virtual void update() override;

    /**
	 * @brief      Helper function to keep track of the constraint evaluation
	 *             used for cppad codegeneration
	 *
	 * @param[in]  stateinput  The stacked state input vector
	 *
	 * @return     The evaluated intermediate constraints
	 */
    Eigen::Matrix<CGScalar, Eigen::Dynamic, 1> evaluateIntermediateCodegen(
        const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM, 1>& stateinput);

    /**
	 * @brief      Helper function to keep track of the constraint evaluation
	 *             used for cppad codegeneration
	 *
	 * @param[in]  stateinput  The stacked state input vector
	 *
	 * @return     The evaluated terminal constraints
	 */
    Eigen::Matrix<CGScalar, Eigen::Dynamic, 1> evaluateTerminalCodegen(
        const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM, 1>& stateinput);

    //containers
    std::vector<std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>>> constraintsIntermediate_;
    std::vector<std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>>> constraintsTerminal_;

    std::shared_ptr<JacCG> intermediateCodegen_;
    std::shared_ptr<JacCG> terminalCodegen_;

    typename JacCG::FUN_TYPE_CG fIntermediate_;
    typename JacCG::FUN_TYPE_CG fTerminal_;

    Eigen::VectorXi sparsityIntermediateRows_;
    Eigen::VectorXi sparsityStateIntermediateRows_;
    Eigen::VectorXi sparsityStateIntermediateCols_;
    Eigen::VectorXi sparsityInputIntermediateRows_;
    Eigen::VectorXi sparsityInputIntermediateCols_;

    Eigen::VectorXi sparsityTerminalRows_;
    Eigen::VectorXi sparsityStateTerminalRows_;
    Eigen::VectorXi sparsityStateTerminalCols_;
    Eigen::VectorXi sparsityInputTerminalRows_;
    Eigen::VectorXi sparsityInputTerminalCols_;


    Eigen::Matrix<SCALAR, STATE_DIM + CONTROL_DIM, 1> stateControlD_; /** contains x, u in stacked form */
};


}  // namespace optcon
}  // namespace ct

#endif
