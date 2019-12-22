/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#ifdef CPPADCG

#include <ct/core/core.h>
#include <memory>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/algorithm/string.hpp>

#include "CostFunctionQuadratic.hpp"
#include "utility/utilities.hpp"

#include "term/TermLoadMacros.hpp"

namespace ct {
namespace optcon {

/**
 * \ingroup CostFunction
 *
 * \brief Cost Function with Auto-Diff support
 *
 * This cost function can work with both, analytical terms as well as
 * auto-diff terms. For analytical terms it will use provided derivatives
 * and for auto-diff terms derivatives will be computed using auto-diff.
 *
 * Unit test \ref ADTest.cpp illustrates the use of a CostFunctionAD.
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class CostFunctionAD : public CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef core::DerivativesCppadJIT<STATE_DIM + CONTROL_DIM + 1, 1> JacCG;
    typedef typename JacCG::CG_SCALAR CGScalar;
    typedef Eigen::Matrix<CGScalar, 1, 1> MatrixCg;

    typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> control_state_matrix_t;

    typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

    /**
	 * \brief Basic constructor
	 */
    CostFunctionAD();

    /**
	 * \brief Constructor loading function from file
	 * @param filename config file location
	 * @param verbose flag enabling printouts
	 */
    CostFunctionAD(const std::string& filename, bool verbose = false);

    /**
	 * Deep-cloning of cost function
	 * @return base pointer to clone
	 */
    CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override;

    /**
	 * \brief Copy constructor
	 * @param arg cost function to copy
	 */
    CostFunctionAD(const CostFunctionAD& arg);


    /**
	 * \brief Destructor
	 */
    virtual ~CostFunctionAD();


    /**
	 * @brief      Initializes the AD costfunction, generates and compiles
	 *             source code
	 */
    virtual void initialize() override;

    /**
	 * \brief Add an intermediate, auto-differentiable term
	 *
	 * Use this function to add an auto-differentiable, intermediate term to the cost function.
	 *
	 * @param term The term to be added
	 * @param verbose Flag enabling printouts
	 * @return
	 */
    void addIntermediateADTerm(std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>> term,
        bool verbose = false) override;

    /**
	 * \brief Add a final, auto-differentiable term
	 *
	 * Use this function to add an auto-differentiable, final term to the cost function.
	 *
	 * @param term The term to be added
	 * @param verbose Flag enabling printouts
	 * @return
	 */
    void addFinalADTerm(std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>> term,
        bool verbose = false) override;

    void setCurrentStateAndControl(const state_vector_t& x, const control_vector_t& u, const SCALAR& t = 0.0) override;

    void loadFromConfigFile(const std::string& filename, bool verbose = false) override;

    SCALAR evaluateIntermediate() override;
    SCALAR evaluateTerminal() override;

    state_vector_t stateDerivativeIntermediate() override;
    state_vector_t stateDerivativeTerminal() override;

    control_vector_t controlDerivativeIntermediate() override;
    control_vector_t controlDerivativeTerminal() override;

    state_matrix_t stateSecondDerivativeIntermediate() override;
    state_matrix_t stateSecondDerivativeTerminal() override;

    control_matrix_t controlSecondDerivativeIntermediate() override;
    control_matrix_t controlSecondDerivativeTerminal() override;

    control_state_matrix_t stateControlDerivativeIntermediate() override;
    control_state_matrix_t stateControlDerivativeTerminal() override;

    std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>> getIntermediateADTermById(const size_t id);

    std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>> getFinalADTermById(const size_t id);

    std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>> getIntermediateADTermByName(
        const std::string& name);

    std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>> getFinalADTermByName(const std::string& name);


private:
    MatrixCg evaluateIntermediateCg(const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1, 1>& stateInputTime);
    MatrixCg evaluateTerminalCg(const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1, 1>& stateInputTime);

    //! combined state, control and time vector
    Eigen::Matrix<SCALAR, STATE_DIM + CONTROL_DIM + 1, 1> stateControlTime_;

    //! intermediate AD terms
    std::vector<std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>>> intermediateTerms_;
    //! final AD terms
    std::vector<std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>>> finalTerms_;

    //! generated jacobians
    std::shared_ptr<JacCG> intermediateCostCodegen_;
    std::shared_ptr<JacCG> finalCostCodegen_;

    //! cppad functions
    typename JacCG::FUN_TYPE_CG intermediateFun_;
    typename JacCG::FUN_TYPE_CG finalFun_;
};

}  // namespace optcon
}  // namespace ct

#endif
