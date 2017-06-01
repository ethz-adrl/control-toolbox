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


#ifndef CT_OPTCON_NLP_NLP_H_
#define CT_OPTCON_NLP_NLP_H_

#include "OptVector.h"
#include "DiscreteConstraintContainerBase.h"

namespace ct {
namespace optcon {


/** @defgroup   NLP NLP
 *
 * @brief      Non linear program Module
 */

/**
 * @ingroup    NLP
 *
 * @brief      The NLP base class. This class serves as abstract base class to
 *             use as an interface to the NLP solver IPOPT and SNOPT
 */
class Nlp
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef double Number;
	typedef Eigen::Matrix<Number, Eigen::Dynamic, 1> VectorXd;
	typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VectorXi;
	typedef Eigen::Map<VectorXd> MapVecXd;
	typedef Eigen::Map<VectorXi> MapVecXi;
	typedef Eigen::Map<const VectorXd> MapConstVecXd;

	/**
	 * @brief      Default constructor
	 */
	Nlp(){}

	/**
	 * @brief      Destructor
	 */
	virtual ~Nlp(){}

	/**
	 * @brief      { This method gets called at each update of the Optimization
	 *             variables. This can be used to distribute or rearrange the
	 *             optimization variables appropriately }
	 */
	virtual void updateProblem() = 0;


	/**
	 * @brief      { Evaluates the costfunction at the current nlp iteration }
	 *
	 * @return     { Scalar value of the resulting cost }
	 */
	Number evaluateCostFun(){
		return costEvaluator_->eval();
	}


	/**
	 * @brief      { Evaluates the gradient of the costfunction}
	 *
	 * @param[in]  n     { size of the gradient }
	 * @param[out] grad  The gradient of the cost function
	 */
	void evaluateCostGradient(const size_t n, MapVecXd& grad){
		costEvaluator_->evalGradient(n, grad);
	} 

	/**
	 * @brief      { Evaluates the constraints }
	 *
	 * @param[out] values  The values of the constraint violations, wrapped as a
	 *                     vector
	 */
	void evaluateConstraints(MapVecXd& values){
		constraints_->evalConstraints(values);
	}

	/**
	 * @brief      { Evaluates the constraint jacobian }
	 *
	 * @param[in]  nele_jac  The number of non zeros in the constraint jacobian
	 * @param[out] jac       The non zero values of the jacobian
	 */
	void evaluateConstraintJacobian(const int nele_jac, MapVecXd& jac){
		constraints_->evalSparseJacobian(jac, nele_jac);
	}

	/**
	 * @brief      Gets the sparsity pattern.
	 *
	 * @param[in]  nele_jac  The number of non zero elements in the constraint
	 *                       jacobian
	 * @param[out] iRow      The row indices of the location of the non zero
	 *                       elements of the constraint jacobian
	 * @param[out] jCol      The column indices of the location of the non zero
	 *                       elements of the constraint jacobian
	 */
	void getSparsityPattern(const int nele_jac, MapVecXi& iRow, MapVecXi& jCol) const{
		constraints_->getSparsityPattern(iRow, jCol, nele_jac);
	}

	/**
	 * @brief      Returns the number of constraints in the NLP
	 *
	 * @return     The number of constraints.
	 */
	size_t getConstraintsCount() const{
		return constraints_->getConstraintsCount();
	}

	/**
	 * @brief      Returns the number of the non zero elements of the constraint
	 *             jacobian.
	 *
	 * @return     The number of the non zero elements of the constraint
	 *             jacobian.
	 */
	size_t getNonZeroJacobianCount() const{
		return constraints_->getNonZerosJacobianCount();
	}

	/**
	 * @brief      Reads the bounds of the constraints.
	 *
	 * @param[out] lowerBound  The lower constraint bound
	 * @param[out] upperBound  The upper constraint bound
	 * @param[in]  m           { The size of the constraints }
	 */
	void getConstraintBounds(MapVecXd& lowerBound, MapVecXd& upperBound, const size_t m) const
	{
		constraints_->getBounds(lowerBound, upperBound);
	}

	/**
	 * @brief      Returns the number of Optimization optimization variables
	 *
	 * @return     The number of Optimization variables.
	 */
	size_t getVarCount() const {return optVariables_->size();}

	/**
	 * @brief      Reads the bounds on the Optimization optimization variables.
	 *
	 * @param[out] lowerBound  The lower optimization variable bound
	 * @param[out] upperBound  The upper optimization variable bound
	 * @param[in]  n           { The number of Optimization variables }
	 */
	void getVariableBounds(MapVecXd& lowerBound, MapVecXd& upperBound, const size_t n) const{
		optVariables_->getLowerBounds(lowerBound);
		optVariables_->getUpperBounds(upperBound);
	}

	/**
	 * @brief      {Extracts the Optimization optimization variables from the nlp
	 *             solvers between nlp iterations}
	 *
	 * @param[in]  x      { The value of the Optimization variables }
	 * @param[in]  isNew  Indicates whether x differs from a previous call
	 */
	void extractOptimizationVars(const MapConstVecXd& x, bool isNew){
		if(isNew)
		{
			optVariables_->setOptimizationVars(x);
			updateProblem();
		}
	}

	/**
	 * @brief      Gets the Optimization variables.
	 *
	 * @param[in]  n     { The number of Optimization variables }
	 * @param[out] x     { The values of the Optimization vars }
	 */
	void getOptimizationVars(const size_t n, MapVecXd& x) const{
		optVariables_->getOptimizationVars(n, x);
	}

	/**
	 * @brief      Gets the variable multiplier and the variable state, used in
	 *             the NLP solver SNOPT. See the snopt documentation for further
	 *             explanations
	 *
	 * @param[in]  n       { The number of Optimization variables  }
	 * @param[out] xMul    The optimization variable multiplier
	 * @param[out] xState  The optimization variable states
	 */
	void getOptimizationMultState(const size_t n, Eigen::Map<Eigen::VectorXd>& xMul, Eigen::Map<Eigen::VectorXi>& xState) const{
		optVariables_->getOptimizationMultState(n, xMul, xState);
	}

	/**
	 * @brief      Gets the constraint multiplier and state, used in the NLP
	 *             solver SNOPT.
	 *
	 * @param[in]  m       { The number of constraints }
	 * @param[out] zMul    The constraint variable multiplier
	 * @param[out] zState  The constraint variable state
	 */
	void getConstraintsMultState(const size_t m, Eigen::Map<Eigen::VectorXd>& zMul, Eigen::Map<Eigen::VectorXi>& zState) const{
		optVariables_->getConstraintsMultState(m, zMul, zState);
	}

	/**
	 * @brief      Gets the bound multipliers used in the NLP solver IPOPT.
	 *
	 * @param[in]  n     { The number of optimization variables }
	 * @param[out] zLow  The value for the lower bound multiplier
	 * @param[out] zUp   The value for the upper bound multiplier
	 */
	void getBoundMultipliers(size_t n, MapVecXd& zLow, MapVecXd& zUp) const{
		optVariables_->getBoundMultipliers(n, zLow, zUp);
	}

	/**
	 * @brief      Gets the values of the constraint multipliers.
	 *
	 * @param[in]  m       { The number of constraints }
	 * @param[out] lambda  The values of the constraint multipliers
	 */
	void getLambdaVars(size_t m, MapVecXd& lambda) const{
		optVariables_->getLambdaVars(m, lambda);
	}

	/**
	 * @brief      { Extracts the solution values from IPOPT }
	 *
	 * @param[in]  x       { The values of the optimization variables }
	 * @param[in]  zL      The value for the lower bound multiplier
	 * @param[in]  zU      The value for the upper bound multiplier
	 * @param[in]  lambda  The values of the constraint multipliers
	 */
	void extractIpoptSolution(const MapConstVecXd& x, const MapConstVecXd& zL, const MapConstVecXd& zU, const MapConstVecXd& lambda) {
		optVariables_->setNewIpoptSolution(x, zL, zU, lambda);
	}

	/**
	 * @brief      { Extracts the solution values from SNOPT }
	 *
	 * @param[in]  x       { The values of the optimization variables }
	 * @param[in]  xMul    The optimization variable multiplier
	 * @param[in]  xState  The optimization variable state
	 * @param[in]  fMul    The constraint variable multiplier
	 * @param[in]  fState  The constraint variable state
	 */
	void extractSnoptSolution(const MapVecXd& x, const MapVecXd& xMul, const MapVecXi& xState, const MapVecXd& fMul, const MapVecXi& fState) {
		optVariables_->setNewSnoptSolution(x, xMul, xState, fMul, fState);
	}


protected:
	std::shared_ptr<DiscreteCostEvaluatorBase> costEvaluator_; //! abstract base class, approximates the cost evaluation for the discrete problem
	std::shared_ptr<OptVector> optVariables_; //! base class, containts the optimization variables used in the NLP solvers
	std::shared_ptr<DiscreteConstraintContainerBase> constraints_; //! abstract base class, contains the discretized constraints for the problem
};

} // namespace optcon
} // namespace ct

#endif //CT_OPTCON_NLP_NLP_H_