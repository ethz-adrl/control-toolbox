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

#include <external/cppad/example/cppad_eigen.hpp>

#include <memory>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/algorithm/string.hpp>   

#include "CostFunctionQuadratic.hpp"
#include "utility/utilities.hpp"

#include "term/TermsAnalytical.hpp"
#include "term/TermsAD.hpp"


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
template <size_t STATE_DIM, size_t CONTROL_DIM>
class CostFunctionAD : public CostFunctionQuadratic<STATE_DIM, CONTROL_DIM> {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> 	state_matrix_t;
	typedef Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
	typedef Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> 	control_state_matrix_t;

	typedef core::StateVector<STATE_DIM> 	 state_vector_t;
	typedef core::ControlVector<CONTROL_DIM> control_vector_t;

	typedef TermBase<STATE_DIM, CONTROL_DIM, CppAD::AD<double>, double> TermBaseAD;

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
	CostFunctionAD(const state_vector_t &x, const control_vector_t &u, const double& t = 0.0);

	/**
	 * \brief Constructor loading function from file
	 * @param filename config file location
	 * @param verbose flag enabling printouts
	 */
	CostFunctionAD(const std::string& filename, bool verbose = false) {
		loadFromConfigFile(filename, verbose);
	}

	/**
	 * Deep-cloning of cost function
	 * @return base pointer to clone
	 */
	CostFunctionAD<STATE_DIM, CONTROL_DIM>* clone () const {
		return new CostFunctionAD(*this);
	}

	/**
	 * \brief Copy constructor
	 * @param arg cost function to copy
	 */
	CostFunctionAD(const CostFunctionAD& arg);


	/**
	 * \brief Destructor
	 */
	~CostFunctionAD() {};

	/**
	 * \brief This function should be called when weightings of a term changed.
	 *
	 * The AD cost function holds an expression of the term function. If weights changes this currently
	 * has to be rebuilt. Call this function to inform the cost function about any changes in parameters
	 * used in the term. This does **NOT** have to be called if x, u or t change.
	 * @param termId The ID of the term that changed
	 */
	void termChanged(size_t termId);

	/**
	 * \brief Add an intermediate, auto-differentiable term
	 *
	 * Use this function to add an auto-differentiable, intermediate term to the cost function.
	 *
	 * @param term The term to be added
	 * @param verbose Flag enabling printouts
	 * @return
	 */
	size_t addIntermediateTerm (std::shared_ptr< TermBaseAD > term, bool verbose = false);

	/**
	 * \brief Add a final, auto-differentiable term
	 *
	 * Use this function to add an auto-differentiable, final term to the cost function.
	 *
	 * @param term The term to be added
	 * @param verbose Flag enabling printouts
	 * @return
	 */
	size_t addFinalTerm (std::shared_ptr< TermBaseAD > term, bool verbose = false);

	size_t addIntermediateTerm (std::shared_ptr< TermBase<STATE_DIM, CONTROL_DIM, double> > term, bool verbose = false) override;
	size_t addFinalTerm (std::shared_ptr< TermBase<STATE_DIM, CONTROL_DIM, double> > term, bool verbose = false) override;

	void setCurrentStateAndControl(const state_vector_t &x, const control_vector_t &u, const double& t = 0.0) override;

	void loadFromConfigFile(const std::string& filename,bool verbose = false) override;

	double evaluateIntermediate() override;
	double evaluateTerminal() override;

	state_vector_t stateDerivativeIntermediate() override;
	state_vector_t stateDerivativeTerminal()     override;

	state_matrix_t stateSecondDerivativeIntermediate() override {return hessian_state_intermediate_;}
	state_matrix_t stateSecondDerivativeTerminal()     override {return hessian_state_final_;}

	control_vector_t controlDerivativeIntermediate() override;
	control_vector_t controlDerivativeTerminal()     override;

	control_matrix_t controlSecondDerivativeIntermediate() override {return hessian_control_intermediate_;}
	control_matrix_t controlSecondDerivativeTerminal()     override {return hessian_control_final_;}

	control_state_matrix_t stateControlDerivativeIntermediate() override {return hessian_control_state_intermediate_;}
	control_state_matrix_t stateControlDerivativeTerminal()     override {return hessian_control_state_final_;}


private:
	//containers
	std::vector<std::shared_ptr< TermBaseAD>> intermediateCostAD_; /** container holding intermediate AD terms **/
	std::vector<std::shared_ptr< TermBaseAD>> finalCostAD_; /** container holding final AD terms **/

	//variables
	Eigen::Matrix<double, Eigen::Dynamic, 1> var_; /** An auto-diff variable **/

	// ADFun does not have a copy constructor. The copy constructor would be called though if we were to use a vector of AdFun directly.
	// Hence, we use vectors of shared_ptrs...
	std::vector<std::shared_ptr<CppAD::ADFun<double>>> intermediateFunctionAD_; /** AD expressions of each intermediate term **/
	std::vector<std::shared_ptr<CppAD::ADFun<double>>> finalFunctionAD_; /** AD expressions of each final term **/

	/**
	 * \brief Records expression of an AD term
	 * @param costAD The AD term
	 * @param functionAD Expression (AD function)
	 */
	void recordTerm(const std::shared_ptr< TermBaseAD > &costAD, CppAD::ADFun<double>& functionAD);


	double evaluateCost(std::vector<std::shared_ptr<TermBaseAD>>& termsAD,
			std::vector<std::shared_ptr<CppAD::ADFun<double>>>& functionAD,
			std::vector<std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, double>>>& termsAnalytical);

	/**
	 * \brief Computing first order derivatives using reverse mode
	 * @param f AD function to compute derivative of
	 * @return Derivative (Jacobian)
	 */
	Eigen::Matrix<double, STATE_DIM + CONTROL_DIM +1, 1> Reverse1(CppAD::ADFun<double> &f);


	/**
	 * \brief Computing first order derivatives using reverse mode for a vector of terms
	 * @param termsAD vector terms to compute derivative of
	 * @param functionAD vector of AD functions to compute derivative of
	 * @param result resulting derivative
	 */
	void reverseADTerms(std::vector< std::shared_ptr< TermBaseAD > > & termsAD,
			std::vector<std::shared_ptr<CppAD::ADFun<double>>> &functionAD,
			Eigen::Matrix<double, STATE_DIM + CONTROL_DIM +1, 1>& result);


	void getHessians(std::vector<std::shared_ptr<TermBaseAD>>& termsAD,
			std::vector<std::shared_ptr<CppAD::ADFun<double>>>& functionAD,
			std::vector<std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, double>>>& termsAnalytical,
			Eigen::Matrix<double, STATE_DIM,   STATE_DIM>&   hessian_state_,
			Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>& hessian_control_,
			Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>&   hessian_control_state_);


	Eigen::Matrix<double, STATE_DIM + CONTROL_DIM +1, 1> reverse1_derivative_intermediate_;
	Eigen::Matrix<double, STATE_DIM + CONTROL_DIM +1, 1> reverse1_derivative_final_;

	Eigen::Matrix<double, STATE_DIM,   STATE_DIM>   hessian_state_intermediate_;
	Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> hessian_control_intermediate_;
	Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>   hessian_control_state_intermediate_;

	Eigen::Matrix<double, STATE_DIM,   STATE_DIM>   hessian_state_final_;
	Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> hessian_control_final_;
	Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>   hessian_control_state_final_;

};

#include "implementation/CostFunctionAD-impl.hpp"

} // namespace optcon
} // namespace ct

#endif // CT_COSTFUNCTIONAD_HPP_
