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
namespace tpl {


/** @defgroup   NLP NLP
 *
 * @brief      The nonlinear program module
 */

/**
 * @ingroup    NLP
 *
 * @brief      The NLP base class. This class serves as abstract base class to
 *             use as an interface to the NLP solver IPOPT and SNOPT
 */
template <typename SCALAR>
class Nlp
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
	typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VectorXi;
	typedef Eigen::Map<VectorXs> MapVecXs;
	typedef Eigen::Map<VectorXi> MapVecXi;
	typedef Eigen::Map<const VectorXs> MapConstVecXs;

	/**
	 * @brief      Default constructor
	 */
	Nlp()
	:
	useGeneratedCostGradient_(false),
	useGeneratedConstraintJacobian_(false)
	{}

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
	SCALAR evaluateCostFun(){
		if(useGeneratedCostGradient_)
			return costCodegen_->forwardZero(optVariables_->getOptimizationVars())(0);
		else
			return costEvaluator_->eval();
			
	}


	/**
	 * @brief      { Evaluates the gradient of the costfunction}
	 *
	 * @param[in]  n     { size of the gradient }
	 * @param[out] grad  The gradient of the cost function
	 */
	void evaluateCostGradient(const size_t n, MapVecXs& grad){
		if(useGeneratedCostGradient_)
			grad = costCodegen_->jacobian(optVariables_->getOptimizationVars());
		else
			costEvaluator_->evalGradient(n, grad);
	} 

	/**
	 * @brief      { Evaluates the constraints }
	 *
	 * @param[out] values  The values of the constraint violations, wrapped as a
	 *                     vector
	 */
	void evaluateConstraints(MapVecXs& values){
		if(useGeneratedConstraintJacobian_)
			values = constraintsCodegen_->forwardZero(optVariables_->getOptimizationVars());
		else
			constraints_->evalConstraints(values);
	}

	/**
	 * @brief      { Evaluates the constraint jacobian }
	 *
	 * @param[in]  nele_jac  The number of non zeros in the constraint jacobian
	 * @param[out] jac       The non zero values of the jacobian
	 */
	void evaluateConstraintJacobian(const int nele_jac, MapVecXs& jac){
		if(useGeneratedConstraintJacobian_)
			jac = constraintsCodegen_->sparseJacobian(optVariables_->getOptimizationVars());
		else
			constraints_->evalSparseJacobian(jac, nele_jac);
	}

	void evaluateHessian(const int nele_hes, MapVecXs& hes, const SCALAR obj_fac, MapConstVecXs& lambda)
	{
		if(useGeneratedConstraintJacobian_ && useGeneratedCostGradient_)
		{
			Eigen::Matrix<double, 1, 1> mat; mat << obj_fac;
			if(nele_hes > 0)
			{
				hes.setZero();

				Eigen::SparseMatrix<double> hessianCost;
				Eigen::SparseMatrix<double> hessianConstraints;
				Eigen::SparseMatrix<double> hessianTotal; 

				hessianCost = costCodegen_->sparseHessian1(optVariables_->getOptimizationVars(), mat);
				hessianConstraints = constraintsCodegen_->sparseHessian1(optVariables_->getOptimizationVars(), lambda);

				hessianTotal = (hessianCost + hessianConstraints).triangularView<Eigen::Lower>();

				hes = Eigen::Map<Eigen::VectorXd>(hessianTotal.valuePtr(), nele_hes, 1);
			}
		}
		else
			throw std::runtime_error("Hessian Evaluation only implemented for codegeneration");
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
	void getSparsityPatternJacobian(const int nele_jac, MapVecXi& iRow, MapVecXi& jCol) const{
		iRow.setZero();
		jCol.setZero();

		if(useGeneratedConstraintJacobian_)
		{
			Eigen::VectorXi iRow1;
			Eigen::VectorXi jCol1;
			constraintsCodegen_->getSparsityPatternJacobian(iRow1, jCol1);

			iRow = iRow1;
			jCol = jCol1;
		}
		else
			constraints_->getSparsityPattern(iRow, jCol, nele_jac);
	}

	void getSparsityPatternHessian(const int nele_hes, MapVecXi& iRow, MapVecXi& jCol) const{

		iRow.setZero();
		jCol.setZero();

		if(useGeneratedConstraintJacobian_ && useGeneratedCostGradient_)
		{
			iRow = iRowHessian_;
			jCol = jColHessian_;
		}
		else
			throw std::runtime_error("Hessian Calculation only available for codegeneration");
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
		if(useGeneratedConstraintJacobian_)
			return constraintsCodegen_->getNumNonZerosJacobian();
		else
			return constraints_->getNonZerosJacobianCount();
	}

	size_t getNonZeroHessianCount() {
		if(useGeneratedConstraintJacobian_)
		{
			Eigen::VectorXi iRowCost;
			Eigen::VectorXi jColCost;
			Eigen::VectorXi iRowCon;
			Eigen::VectorXi jColCon;
			costCodegen_->getSparsityPatternHessian(iRowCost, jColCost);
			// std::cout << "iRowCost: " << iRowCost.transpose() << std::endl;
			// std::cout << "jColCost: " << jColCost.transpose() << std::endl;
			constraintsCodegen_->getSparsityPatternHessian(iRowCon, jColCon);
			// std::cout << "iRowCon: " << iRowConstraints.transpose() << std::endl;
			// std::cout << "jColCon: " << jColConstraints.transpose() << std::endl;

			std::vector<int> iRowHessianLocal;
			std::vector<int> jColHessianLocal;
			size_t i_cost, i_con;
			i_cost = 0;
			i_con = 0;

			while((i_cost < iRowCost.rows()) && (i_con < iRowCon.rows()))
			{
				if(iRowCost(i_cost) == iRowCon(i_con))
				{
					if(jColCost(i_cost) == jColCon(i_con))
					{
						iRowHessianLocal.push_back(iRowCost(i_cost));
						jColHessianLocal.push_back(jColCost(i_cost));
						i_cost++;
						i_con++;
					}
					else if(jColCost(i_cost) < jColCon(i_con))
					{
						iRowHessianLocal.push_back(iRowCost(i_cost));
						jColHessianLocal.push_back(jColCost(i_cost));
						i_cost++;
					}
					else if(jColCost(i_cost) > jColCon(i_con))
					{
						iRowHessianLocal.push_back(iRowCon(i_con));
						jColHessianLocal.push_back(jColCon(i_con));
						i_con++;
					}
				}
				else if(iRowCost(i_cost) < iRowCon(i_con))
				{
					iRowHessianLocal.push_back(iRowCost(i_cost));
					jColHessianLocal.push_back(jColCost(i_cost));
					i_cost++;
				}
				else if(iRowCost(i_cost) > iRowCon(i_con))
				{
					iRowHessianLocal.push_back(iRowCon(i_con));
					jColHessianLocal.push_back(jColCon(i_con));
					i_con++;
				}
			}

			if(i_cost < iRowCost.rows())
			{
				for(size_t t = i_cost; t < iRowCost.rows(); ++t)
				{
					iRowHessianLocal.push_back(iRowCost(t));
					jColHessianLocal.push_back(jColCost(t));
				}
			}
			else if(i_con < iRowCon.rows())
			{
				for(size_t t = i_con; t < iRowCon.rows(); ++t)
				{
					iRowHessianLocal.push_back(iRowCon(t));
					jColHessianLocal.push_back(jColCon(t));
				}
			}

			iRowHessian_ = Eigen::Map<Eigen::VectorXi>(iRowHessianLocal.data(), iRowHessianLocal.size(), 1);
			jColHessian_ = Eigen::Map<Eigen::VectorXi>(jColHessianLocal.data(), jColHessianLocal.size(), 1);

			std::cout << "iRowHessian_: " << iRowHessian_.transpose() << std::endl;
			std::cout << "jColHessian_: " << jColHessian_.transpose() << std::endl;

			size_t nonZerosHessian = iRowHessian_.rows();
			return nonZerosHessian;
		}
		else
			throw std::runtime_error("Get nonzeros hessian only valid for codegenerated derivatives");
	}	

	/**
	 * @brief      Reads the bounds of the constraints.
	 *
	 * @param[out] lowerBound  The lower constraint bound
	 * @param[out] upperBound  The upper constraint bound
	 * @param[in]  m           { The size of the constraints }
	 */
	void getConstraintBounds(MapVecXs& lowerBound, MapVecXs& upperBound, const size_t m) const
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
	void getVariableBounds(MapVecXs& lowerBound, MapVecXs& upperBound, const size_t n) const{
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
	void extractOptimizationVars(const MapConstVecXs& x, bool isNew){
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
	void getOptimizationVars(const size_t n, MapVecXs& x) const{
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
	void getOptimizationMultState(const size_t n, MapVecXs& xMul, MapVecXi& xState) const{
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
	void getConstraintsMultState(const size_t m, MapVecXs& zMul, MapVecXi& zState) const{
		optVariables_->getConstraintsMultState(m, zMul, zState);
	}

	/**
	 * @brief      Gets the bound multipliers used in the NLP solver IPOPT.
	 *
	 * @param[in]  n     { The number of optimization variables }
	 * @param[out] zLow  The value for the lower bound multiplier
	 * @param[out] zUp   The value for the upper bound multiplier
	 */
	void getBoundMultipliers(size_t n, MapVecXs& zLow, MapVecXs& zUp) const{
		optVariables_->getBoundMultipliers(n, zLow, zUp);
	}

	/**
	 * @brief      Gets the values of the constraint multipliers.
	 *
	 * @param[in]  m       { The number of constraints }
	 * @param[out] lambda  The values of the constraint multipliers
	 */
	void getLambdaVars(size_t m, MapVecXs& lambda) const{
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
	void extractIpoptSolution(const MapConstVecXs& x, const MapConstVecXs& zL, const MapConstVecXs& zU, const MapConstVecXs& lambda) {
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
	void extractSnoptSolution(const MapVecXs& x, const MapVecXs& xMul, const MapVecXi& xState, const MapVecXs& fMul, const MapVecXi& fState) {
		optVariables_->setNewSnoptSolution(x, xMul, xState, fMul, fState);
	}


protected:
	std::shared_ptr<DiscreteCostEvaluatorBase<SCALAR>> costEvaluator_; //! abstract base class, approximates the cost evaluation for the discrete problem
	std::shared_ptr<OptVector<SCALAR>> optVariables_; //! base class, containts the optimization variables used in the NLP solvers
	std::shared_ptr<DiscreteConstraintContainerBase<SCALAR>> constraints_; //! abstract base class, contains the discretized constraints for the problem
	bool useGeneratedCostGradient_;
	bool useGeneratedConstraintJacobian_;
	std::shared_ptr<ct::core::DerivativesCppad<-1, 1>> costCodegen_;
	std::shared_ptr<ct::core::DerivativesCppad<-1, -1>> constraintsCodegen_;
	Eigen::VectorXi iRowHessian_;
	Eigen::VectorXi jColHessian_;

	// bool pairCompair(const std::pair<int, int>& firstElement, const std::pair<int, int>& secondElement)
	// {
	// 	bool isFirstEleSmaller = firstElement.first < secondElement.first;
	// 	return firstElement.first < secondElement.first && firstElement.second < secondElement.second;
	// }
};

}

typedef tpl::Nlp<double> Nlp;

} // namespace optcon
} // namespace ct

#endif //CT_OPTCON_NLP_NLP_H_