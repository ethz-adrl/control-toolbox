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

	// This function gets called when a new set of variables is provided by the NLPSolver
	// Define this function for your NLP
	virtual void updateProblem() = 0;


	Number evaluateCostFun(){
		return costEvaluator_->eval();
	}

	void evaluateCostGradient(size_t n, MapVecXd& grad){
		costEvaluator_->evalGradient(n, grad);
	} 

	void evaluateConstraints(MapVecXd& values){
		constraints_->evalConstraints(values);
	}

	void evaluateConstraintJacobian(const int nele_jac, MapVecXd& jac){
		constraints_->evalSparseJacobian(jac, nele_jac);
	}

	void getSparsityPattern(const int nele_jac, MapVecXi& iRow, MapVecXi& jCol) const{
		constraints_->getSparsityPattern(iRow, jCol, nele_jac);
	}

	// Maybe the virtual can be bypassed
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

	void setPrimalVars(const MapConstVecXd& x, bool isNew){
		optVariables_->setPrimalVars(x, isNew);
		if(isNew)
			updateProblem();
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

	void getInitBoundMultipliers(size_t n, MapVecXd& zLow, MapVecXd& zUp) const{
		optVariables_->getInitBoundMultipliers(n, zLow, zUp);
	}

	void getInitLambdaVars(size_t m, MapVecXd& lambda) const{
		optVariables_->getInitLambdaVars(m, lambda);
	}

	void extractSolution(const MapConstVecXd& x, const MapConstVecXd& zL, const MapConstVecXd& zU, const MapConstVecXd& lambda) {
		optVariables_->setNewSolution(x, zL, zU, lambda);
	}

	void extractSolution(const MapVecXd& x, const MapVecXd& xMul, const MapVecXi& xState, const MapVecXd& fMul, const MapVecXi& fState) {
		optVariables_->setNewSolution(x, xMul, xState, fMul, fState);
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