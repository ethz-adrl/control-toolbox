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

#ifndef CT_OPTCON_COSTFUNCTIONAD_HPP_
#define CT_OPTCON_COSTFUNCTIONAD_HPP_

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
class CostFunctionAD : public CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef core::DerivativesCppadJIT<STATE_DIM + CONTROL_DIM + 1, 1> JacCG;
	typedef typename JacCG::CG_SCALAR CGScalar;
	typedef Eigen::Matrix<CGScalar, 1, 1> MatrixCg;

	typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> 	state_matrix_t;
	typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
	typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> 	control_state_matrix_t;

	typedef core::StateVector<STATE_DIM, SCALAR> 	 state_vector_t;
	typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

	/**
	 * \brief Basic constructor
	 */
	CostFunctionAD();

	/**
	 * \brief Constructor using state, control and time
	 * @param x state vector
	 * @param u control vector
	 * @param t time
	 */
	CostFunctionAD(const state_vector_t &x, const control_vector_t &u, const SCALAR& t = 0.0);

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
	CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>* clone () const;

	/**
	 * \brief Copy constructor
	 * @param arg cost function to copy
	 */
	CostFunctionAD(const CostFunctionAD& arg);


	/**
	 * \brief Destructor
	 */
	~CostFunctionAD();


	/**
	 * @brief      Initializes the AD costfunction, generates and compiles
	 *             source code
	 */
	void initialize();

	/**
	 * \brief Add an intermediate, auto-differentiable term
	 *
	 * Use this function to add an auto-differentiable, intermediate term to the cost function.
	 *
	 * @param term The term to be added
	 * @param verbose Flag enabling printouts
	 * @return
	 */
	void addIntermediateADTerm (std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>> term, bool verbose = false) override;

	/**
	 * \brief Add a final, auto-differentiable term
	 *
	 * Use this function to add an auto-differentiable, final term to the cost function.
	 *
	 * @param term The term to be added
	 * @param verbose Flag enabling printouts
	 * @return
	 */
	void addFinalADTerm (std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>> term, bool verbose = false) override;

	void setCurrentStateAndControl(const state_vector_t &x, const control_vector_t &u, const SCALAR& t = 0.0) override;

	void loadFromConfigFile(const std::string& filename, bool verbose = false) override;

	SCALAR evaluateIntermediate() override;
	SCALAR evaluateTerminal() override;

	state_vector_t stateDerivativeIntermediate() override;
	state_vector_t stateDerivativeTerminal()     override;

	control_vector_t controlDerivativeIntermediate() override;
	control_vector_t controlDerivativeTerminal()     override;

	state_matrix_t stateSecondDerivativeIntermediate() override;
	state_matrix_t stateSecondDerivativeTerminal()     override;

	control_matrix_t controlSecondDerivativeIntermediate() override;
	control_matrix_t controlSecondDerivativeTerminal()     override;

	control_state_matrix_t stateControlDerivativeIntermediate() override;
	control_state_matrix_t stateControlDerivativeTerminal()     override;


private:
	MatrixCg evaluateIntermediateCg(const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1, 1>& stateInputTime);
	MatrixCg evaluateTerminalCg(const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1, 1>& stateInputTime);

	Eigen::Matrix<SCALAR, STATE_DIM + CONTROL_DIM + 1, 1> stateControlTime_;

	std::vector<std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>>> intermediateTerms_;
	std::vector<std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>>> finalTerms_;

	std::shared_ptr<JacCG> intermediateCostCodegen_;
	std::shared_ptr<JacCG> finalCostCodegen_;

	typename JacCG::FUN_TYPE_CG intermediateFun_;
	typename JacCG::FUN_TYPE_CG finalFun_;
};

} // namespace optcon
} // namespace ct

#endif // CT_COSTFUNCTIONAD_HPP_
