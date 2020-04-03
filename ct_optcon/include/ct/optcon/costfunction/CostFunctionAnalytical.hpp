/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

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
 * \brief A cost function which contains only terms that have analytical derivatives
 *
 * This class provides functions to evaluate a cost function and computes its first
 * and second order derivatives. This cost function assumes that analytical derivatives
 * for all terms are available.
 */
template <typename MANIFOLD, size_t CONTROL_DIM>
class CostFunctionAnalytical : public CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr size_t STATE_DIM = MANIFOLD::TangentDim;

    using Base = CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>;
    using SCALAR_EVAL = typename Base::SCALAR_EVAL;

    using state_matrix_t = typename Base::state_matrix_t;
    using control_matrix_t = typename Base::control_matrix_t;
    using control_state_matrix_t = typename Base::control_state_matrix_t;
    typedef typename Base::control_vector_t control_vector_t;


    /**
	 * \brief Basic constructor
	 */
    CostFunctionAnalytical();

    /**
	 * \brief Copy constructor
	 * @param arg cost function to copy
	 */
    CostFunctionAnalytical(const CostFunctionAnalytical& arg);
    /**
	 * \brief Constructor loading function from file
	 * @param filename config file location
	 * @param verbose flag enabling printouts
	 */
    CostFunctionAnalytical(const std::string& filename, bool verbose = false);

    /**
	 * Deep-cloning of cost function
	 * @return base pointer to clone
	 */
    CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>* clone() const override;

    /**
	 * Destructor
	 */
    ~CostFunctionAnalytical();

    SCALAR_EVAL evaluateIntermediate() override;
    SCALAR_EVAL evaluateTerminal() override;

    ct::core::StateVector<STATE_DIM, SCALAR_EVAL> stateDerivativeIntermediate() override;
    ct::core::StateVector<STATE_DIM, SCALAR_EVAL> stateDerivativeTerminal() override;

    state_matrix_t stateSecondDerivativeIntermediate() override;
    state_matrix_t stateSecondDerivativeTerminal() override;

    control_vector_t controlDerivativeIntermediate() override;
    control_vector_t controlDerivativeTerminal() override;

    control_matrix_t controlSecondDerivativeIntermediate() override;
    control_matrix_t controlSecondDerivativeTerminal() override;

    control_state_matrix_t stateControlDerivativeIntermediate() override;
    control_state_matrix_t stateControlDerivativeTerminal() override;

    void loadFromConfigFile(const std::string& filename, bool verbose = false) override;

private:
};

}  // namespace optcon
}  // namespace ct
