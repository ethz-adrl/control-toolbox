#ifndef CT_OPTCON_NLP_NLP_H_
#define CT_OPTCON_NLP_NLP_H_

#include "OptVector.h"
#include "DiscreteConstraintContainerBase.h"

namespace ct {
namespace optcon {

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

	Nlp(){}

	virtual ~Nlp(){}

	/**
	 * @brief      { This function gets called at each update of the primal variables }
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

	void evaluateCostGradient(const size_t n, MapVecXd& grad){
		costEvaluator_->evalGradient(n, grad);
	} 

	/**
	 * @brief      { function_description }
	 *
	 * @param      values  The values
	 */
	void evaluateConstraints(MapVecXd& values){
		constraints_->evalConstraints(values);
	}

	/**
	 * @brief      { function_description }
	 *
	 * @param[in]  nele_jac  The nele jac
	 * @param      jac       The jac
	 */
	void evaluateConstraintJacobian(const int nele_jac, MapVecXd& jac){
		constraints_->evalSparseJacobian(jac, nele_jac);
	}

	void getSparsityPattern(const int nele_jac, MapVecXi& iRow, MapVecXi& jCol) const{
		constraints_->getSparsityPattern(iRow, jCol, nele_jac);
	}

	size_t getConstraintsCount() const{
		return constraints_->getConstraintsCount();
	}

	size_t getNonZeroJacobianCount() const{
		return constraints_->getNonZerosJacobianCount();
	}

	void getConstraintBounds(MapVecXd& lowerBound, MapVecXd& upperBound, size_t n) const
	{
		lowerBound = constraints_->getLowerBounds();
		upperBound = constraints_->getUpperBounds();
	}

	size_t getVarCount() const {return optVariables_->size();}

	void getVariableBounds(MapVecXd& lowerBound, MapVecXd& upperBound, size_t n) const{
		optVariables_->getLowerBounds(lowerBound);
		optVariables_->getUpperBounds(upperBound);
	}

	void extractPrimalVars(const MapConstVecXd& x, bool isNew){
		if(isNew)
		{
			optVariables_->setPrimalVars(x);
			updateProblem();
		}
	}

	void getPrimalVars(const size_t n, MapVecXd& x) const{
		optVariables_->getPrimalVars(n, x);
	}

	void getPrimalMultState(const size_t n, Eigen::Map<Eigen::VectorXd>& xMul, Eigen::Map<Eigen::VectorXi>& xState) const{
		optVariables_->getPrimalMultState(n, xMul, xState);
	}

	void getDualMultState(const size_t m, Eigen::Map<Eigen::VectorXd>& zMul, Eigen::Map<Eigen::VectorXi>& zState) const{
		optVariables_->getDualMultState(m, zMul, zState);
	}

	void getBoundMultipliers(size_t n, MapVecXd& zLow, MapVecXd& zUp) const{
		optVariables_->getBoundMultipliers(n, zLow, zUp);
	}

	void getLambdaVars(size_t m, MapVecXd& lambda) const{
		optVariables_->getLambdaVars(m, lambda);
	}

	void extractIpoptSolution(const MapConstVecXd& x, const MapConstVecXd& zL, const MapConstVecXd& zU, const MapConstVecXd& lambda) {
		optVariables_->setNewIpoptSolution(x, zL, zU, lambda);
	}

	void extractSnoptSolution(const MapVecXd& x, const MapVecXd& xMul, const MapVecXi& xState, const MapVecXd& fMul, const MapVecXi& fState) {
		optVariables_->setNewSnoptSolution(x, xMul, xState, fMul, fState);
	}


protected:
	// The finite dimensional vector of decision variables
	std::shared_ptr<DiscreteCostEvaluatorBase> costEvaluator_;
	std::shared_ptr<OptVector> optVariables_;
	// The finite dimensional vector of constraints
	std::shared_ptr<DiscreteConstraintContainerBase> constraints_;
};

}
}




#endif //CT_OPTCON_NLP_NLP_H_