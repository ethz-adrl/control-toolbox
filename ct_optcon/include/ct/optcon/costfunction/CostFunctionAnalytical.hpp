/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#ifndef CT_OPTCON_COSTFUNCTIONANALYTICAL_HPP_
#define CT_OPTCON_COSTFUNCTIONANALYTICAL_HPP_

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/algorithm/string.hpp>   

#include "CostFunctionQuadratic.hpp"
#include "utility/utilities.hpp"

#include "term/TermsAnalytical.hpp"


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
class CostFunctionAnalytical : public CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR> {

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
	CostFunctionAnalytical() {};

	/**
	 * \brief Constructor using state, control and time
	 * @param x state vector
	 * @param u control vector
	 * @param t time
	 */
	CostFunctionAnalytical(const state_vector_t &x, const control_vector_t &u, const SCALAR& t = 0.0) :
		CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>(x, u, t)
	{};

	/**
	 * \brief Copy constructor
	 * @param arg cost function to copy
	 */
	CostFunctionAnalytical(const CostFunctionAnalytical& arg):
		CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>(arg){}

	/**
	 * \brief Constructor loading function from file
	 * @param filename config file location
	 * @param verbose flag enabling printouts
	 */
	CostFunctionAnalytical(const std::string& filename, bool verbose = false) {
		loadFromConfigFile(filename, verbose);
	}

	/**
	 * Deep-cloning of cost function
	 * @return base pointer to clone
	 */
	CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>* clone () const {
		return new CostFunctionAnalytical(*this);
	}

	/**
	 * Destructor
	 */
	~CostFunctionAnalytical() {};

	size_t addIntermediateTerm (std::shared_ptr< TermBase<STATE_DIM, CONTROL_DIM, SCALAR> > term, bool verbose = false) override;
	size_t addFinalTerm (std::shared_ptr< TermBase<STATE_DIM, CONTROL_DIM, SCALAR> > term, bool verbose = false) override;

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
	
	void loadFromConfigFile(const std::string& filename,bool verbose = false) override;

private:
};

#include "implementation/CostFunctionAnalytical.hpp"

} // namespace optcon
} // namespace cf

#endif // COSTFUNCTIONANALYTICAL_HPP_
