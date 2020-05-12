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
template <typename AD_MANIFOLD, size_t CONTROL_DIM>
class CostFunctionAD
    : public CostFunctionQuadratic<
          typename AD_MANIFOLD::template RedefineScalar<typename ct::core::get_out_type<AD_MANIFOLD>::type>,
          CONTROL_DIM>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr size_t STATE_DIM = AD_MANIFOLD::TangentDim;

    using EVAL_MANIFOLD =
        typename AD_MANIFOLD::template RedefineScalar<typename ct::core::get_out_type<AD_MANIFOLD>::type>;
    using Base = CostFunctionQuadratic<EVAL_MANIFOLD, CONTROL_DIM>;

    typedef core::DerivativesCppadJIT<STATE_DIM + CONTROL_DIM + 1, 1> JacCG;

    typedef typename JacCG::CG_SCALAR CGScalar;
    static_assert(std::is_same<typename AD_MANIFOLD::Scalar, CGScalar>::value,
        "scalar types do not match- costfunction AD works only with ADCGScalar.");
    typedef Eigen::Matrix<CGScalar, 1, 1> MatrixCg;

    using state_matrix_t = typename Base::state_matrix_t;
    using control_matrix_t = typename Base::control_matrix_t;
    using control_state_matrix_t = typename Base::control_state_matrix_t;
    typedef typename Base::control_vector_t control_vector_t;

    using SCALAR = typename AD_MANIFOLD::Scalar;  // todo is this the right scalar?
    using SCALAR_EVAL = typename Base::SCALAR_EVAL;
    typedef Eigen::Matrix<SCALAR_EVAL, Eigen::Dynamic, 1> VectorXs;
    typedef Eigen::Matrix<SCALAR_EVAL, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

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
    CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>* clone() const override;

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
    void addIntermediateADTerm(std::shared_ptr<TermBase<AD_MANIFOLD, CONTROL_DIM>> term, bool verbose = false);

    /**
	 * \brief Add a final, auto-differentiable term
	 *
	 * Use this function to add an auto-differentiable, final term to the cost function.
	 *
	 * @param term The term to be added
	 * @param verbose Flag enabling printouts
	 * @return
	 */
    void addFinalADTerm(std::shared_ptr<TermBase<AD_MANIFOLD, CONTROL_DIM>> term, bool verbose = false);

    void setCurrentStateAndControl(const EVAL_MANIFOLD& x,
        const control_vector_t& u,
        const SCALAR_EVAL& t = 0.0) override;

    void loadFromConfigFile(const std::string& filename, bool verbose = false) override;

    SCALAR_EVAL evaluateIntermediate() override;
    SCALAR_EVAL evaluateTerminal() override;

    typename EVAL_MANIFOLD::Tangent stateDerivativeIntermediate() override;
    typename EVAL_MANIFOLD::Tangent stateDerivativeTerminal() override;

    control_vector_t controlDerivativeIntermediate() override;
    control_vector_t controlDerivativeTerminal() override;

    state_matrix_t stateSecondDerivativeIntermediate() override;
    state_matrix_t stateSecondDerivativeTerminal() override;

    control_matrix_t controlSecondDerivativeIntermediate() override;
    control_matrix_t controlSecondDerivativeTerminal() override;

    control_state_matrix_t stateControlDerivativeIntermediate() override;
    control_state_matrix_t stateControlDerivativeTerminal() override;

    std::shared_ptr<TermBase<AD_MANIFOLD, CONTROL_DIM>> getIntermediateADTermById(const size_t id);

    std::shared_ptr<TermBase<AD_MANIFOLD, CONTROL_DIM>> getFinalADTermById(const size_t id);

    std::shared_ptr<TermBase<AD_MANIFOLD, CONTROL_DIM>> getIntermediateADTermByName(const std::string& name);

    std::shared_ptr<TermBase<AD_MANIFOLD, CONTROL_DIM>> getFinalADTermByName(const std::string& name);


private:
    MatrixCg evaluateIntermediateCg(const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1, 1>& stateInputTime);
    MatrixCg evaluateTerminalCg(const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1, 1>& stateInputTime);

    //! combined state, control and time vector
    Eigen::Matrix<SCALAR_EVAL, STATE_DIM + CONTROL_DIM + 1, 1> stateControlTime_;

    //! intermediate AD terms
    std::vector<std::shared_ptr<TermBase<AD_MANIFOLD, CONTROL_DIM>>> intermediateTerms_;
    //! final AD terms
    std::vector<std::shared_ptr<TermBase<AD_MANIFOLD, CONTROL_DIM>>> finalTerms_;

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
