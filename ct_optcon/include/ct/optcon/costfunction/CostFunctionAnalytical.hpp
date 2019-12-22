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
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class CostFunctionAnalytical : public CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> control_state_matrix_t;

    typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;

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
    CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override;

    /**
	 * Destructor
	 */
    ~CostFunctionAnalytical();

    SCALAR evaluateIntermediate() override;
    SCALAR evaluateTerminal() override;

    state_vector_t stateDerivativeIntermediate() override;
    state_vector_t stateDerivativeTerminal() override;

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
