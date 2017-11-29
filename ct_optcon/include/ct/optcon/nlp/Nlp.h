/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#pragma once

#include <Eigen/Sparse>
#include <ct/core/math/DerivativesCppadJIT.h>
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
    Nlp() {}
    /**
	 * @brief      Destructor
	 */
    virtual ~Nlp() {}
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
    SCALAR evaluateCostFun()
    {
        if (!costCodegen_ && !costEvaluator_)
            throw std::runtime_error("Error in evaluateCostFun. Costevaluator not initialized");

        if (costCodegen_)
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
    void evaluateCostGradient(const size_t n, MapVecXs& grad)
    {
        if (!costCodegen_ && !costEvaluator_)
            throw std::runtime_error("Error in evaluateCostGradient. Costevaluator not initialized");

        if (costCodegen_)
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
    void evaluateConstraints(MapVecXs& values)
    {
        if (!constraintsCodegen_ && !constraints_)
            throw std::runtime_error("Error in evaluateConstraints. Constraints not initialized");

        if (constraintsCodegen_)
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
    void evaluateConstraintJacobian(const int nele_jac, MapVecXs& jac)
    {
        if (!constraintsCodegen_ && !constraints_)
            throw std::runtime_error("Error in evaluateConstraintJacobian. Constraints not initialized");

        if (constraintsCodegen_)
            jac = constraintsCodegen_->sparseJacobianValues(optVariables_->getOptimizationVars());
        else
            constraints_->evalSparseJacobian(jac, nele_jac);
    }

    /**
	 * @brief      Evaluates the hessian of the lagrangian
	 *
	 * @param[in]  nele_hes  The number of non zeros in the hessian
	 * @param      hes       The values of the non-zeros of the hessian
	 * @param[in]  obj_fac   The costfunction multiplier
	 * @param      lambda    The constraint multipliers
	 */
    void evaluateHessian(const int nele_hes, MapVecXs& hes, const SCALAR obj_fac, MapConstVecXs& lambda)
    {
        if (!constraintsCodegen_ || !costCodegen_)
            throw std::runtime_error(
                "Error in evaluateHessian. Hessian Evaluation only implemented for codegeneration");

        hes.setZero();
        Eigen::Matrix<double, 1, 1> omega;
        omega << obj_fac;

        Eigen::VectorXd hessianCostValues =
            costCodegen_->sparseHessianValues(optVariables_->getOptimizationVars(), omega);
        Eigen::VectorXd hessianConstraintsValues =
            constraintsCodegen_->sparseHessianValues(optVariables_->getOptimizationVars(), lambda);

        for (size_t i = 0; i < hessianCostValues.rows(); ++i)
            hessianCost_.coeffRef(iRowHessianCost_(i), jColHessianCost_(i)) = hessianCostValues(i);

        for (size_t i = 0; i < hessianConstraintsValues.rows(); ++i)
            hessianConstraints_.coeffRef(iRowHessianConstraints_(i), jColHessianConstraints_(i)) =
                hessianConstraintsValues(i);

        hessianTotal_ = (hessianCost_ + hessianConstraints_).template triangularView<Eigen::Lower>();

        hes = Eigen::Map<Eigen::VectorXd>(hessianTotal_.valuePtr(), nele_hes, 1);
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
    void getSparsityPatternJacobian(const int nele_jac, MapVecXi& iRow, MapVecXi& jCol) const
    {
        if (!constraintsCodegen_ && !constraints_)
            throw std::runtime_error("Error in getSparsityPatternJacobian. Constraints not initialized");

        iRow.setZero();
        jCol.setZero();

        if (constraintsCodegen_)
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

    /**
	 * @brief      Gets the sparsity pattern hessian of the lagrangian
	 *
	 * @param[in]  nele_hes  The number of non zero elements in the hessian
	 * @param      iRow      The row indices
	 * @param      jCol      The column indices
	 */
    void getSparsityPatternHessian(const int nele_hes, MapVecXi& iRow, MapVecXi& jCol) const
    {
        if (!constraintsCodegen_ || !costCodegen_)
            throw std::runtime_error(
                "Error in getSparsityPatternHessian. Hessian Evaluation only implemented for codegeneration");

        iRow.setZero();
        jCol.setZero();

        iRow = iRowHessian_;
        jCol = jColHessian_;
    }

    /**
	 * @brief      Returns the number of constraints in the NLP
	 *
	 * @return     The number of constraints.
	 */
    size_t getConstraintsCount() const
    {
        if (!constraints_)
            throw std::runtime_error("Error in getConstraintsCount. Constraints not initialized");

        return constraints_->getConstraintsCount();
    }

    /**
	 * @brief      Returns the number of the non zero elements of the constraint
	 *             jacobian.
	 *
	 * @return     The number of the non zero elements of the constraint
	 *             jacobian.
	 */
    size_t getNonZeroJacobianCount() const
    {
        if (!constraintsCodegen_ && !constraints_)
            throw std::runtime_error("Error in getNonZeroJacobianCount. Constraints not initialized");

        if (constraintsCodegen_)
            return constraintsCodegen_->getNumNonZerosJacobian();
        else
            return constraints_->getNonZerosJacobianCount();
    }

    /**
	 * @brief      Returns the number of non zeros in the hessian
	 *
	 * @return     The number of non zeros in the hessian
	 */
    size_t getNonZeroHessianCount()
    {
        if (!constraintsCodegen_ || !costCodegen_)
        {
            std::cerr << "Error: Exact hessians only work with AD hessian codegeneration" << std::endl;
            throw std::runtime_error("Error in getNonZeroHessianCount. Codegeneration not initialized");
        }

        costCodegen_->getSparsityPatternHessian(iRowHessianCost_, jColHessianCost_);
        constraintsCodegen_->getSparsityPatternHessian(iRowHessianConstraints_, jColHessianConstraints_);

        std::vector<Eigen::Triplet<SCALAR>> tripletsCost;
        std::vector<Eigen::Triplet<SCALAR>> tripletsConstraints;

        for (size_t i = 0; i < iRowHessianCost_.rows(); ++i)
            tripletsCost.push_back(Eigen::Triplet<SCALAR>(iRowHessianCost_(i), jColHessianCost_(i), SCALAR(0.1)));

        for (size_t i = 0; i < iRowHessianConstraints_.rows(); ++i)
            tripletsConstraints.push_back(
                Eigen::Triplet<SCALAR>(iRowHessianConstraints_(i), jColHessianConstraints_(i), SCALAR(0.1)));

        hessianCost_.resize(optVariables_->size(), optVariables_->size());
        hessianCost_.setFromTriplets(tripletsCost.begin(), tripletsCost.end());
        hessianConstraints_.resize(optVariables_->size(), optVariables_->size());
        hessianConstraints_.setFromTriplets(tripletsConstraints.begin(), tripletsConstraints.end());

        hessianTotal_.resize(optVariables_->size(), optVariables_->size());
        hessianTotal_ = (hessianCost_ + hessianConstraints_).template triangularView<Eigen::Lower>();

        std::vector<int> iRowHessianStdVec;
        std::vector<int> jColHessianStdVec;
        for (size_t k = 0; k < hessianTotal_.outerSize(); ++k)
            for (typename Eigen::SparseMatrix<SCALAR>::InnerIterator it(hessianTotal_, k); it; ++it)
            {
                iRowHessianStdVec.push_back(it.row());
                jColHessianStdVec.push_back(it.col());
            }

        iRowHessian_ = Eigen::Map<Eigen::VectorXi>(iRowHessianStdVec.data(), iRowHessianStdVec.size(), 1);
        jColHessian_ = Eigen::Map<Eigen::VectorXi>(jColHessianStdVec.data(), jColHessianStdVec.size(), 1);

        size_t nonZerosHessian = iRowHessian_.rows();
        return nonZerosHessian;
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
        if (!constraints_)
            throw std::runtime_error("Error in getConstraintBounds. Constraints not initialized");

        constraints_->getBounds(lowerBound, upperBound);
    }

    /**
	 * @brief      Returns the number of Optimization optimization variables
	 *
	 * @return     The number of Optimization variables.
	 */
    size_t getVarCount() const
    {
        if (!optVariables_)
            throw std::runtime_error("Error in getVarCount. Optvariables not initialized");

        return optVariables_->size();
    }

    /**
	 * @brief      Reads the bounds on the Optimization optimization variables.
	 *
	 * @param[out] lowerBound  The lower optimization variable bound
	 * @param[out] upperBound  The upper optimization variable bound
	 * @param[in]  n           { The number of Optimization variables }
	 */
    void getVariableBounds(MapVecXs& lowerBound, MapVecXs& upperBound, const size_t n) const
    {
        if (!optVariables_)
            throw std::runtime_error("Error in getVariableBounds. Optvariables not initialized");

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
    void extractOptimizationVars(const MapConstVecXs& x, bool isNew)
    {
        if (!optVariables_)
            throw std::runtime_error("Error in extractOptimizationVars. Optvariables not initialized");

        if (isNew)
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
    void getInitialGuess(const size_t n, MapVecXs& x) const
    {
        if (!optVariables_)
            throw std::runtime_error("Error in getOptimizationVars. Optvariables not initialized");

        optVariables_->getInitialGuess(n, x);
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
    void getOptimizationMultState(const size_t n, MapVecXs& xMul, MapVecXi& xState) const
    {
        if (!optVariables_)
            throw std::runtime_error("Error in getOptimizationMultState. Optvariables not initialized");

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
    void getConstraintsMultState(const size_t m, MapVecXs& zMul, MapVecXi& zState) const
    {
        if (!optVariables_)
            throw std::runtime_error("Error in getConstraintsMultState. Optvariables not initialized");

        optVariables_->getConstraintsMultState(m, zMul, zState);
    }

    /**
	 * @brief      Gets the bound multipliers used in the NLP solver IPOPT.
	 *
	 * @param[in]  n     { The number of optimization variables }
	 * @param[out] zLow  The value for the lower bound multiplier
	 * @param[out] zUp   The value for the upper bound multiplier
	 */
    void getBoundMultipliers(size_t n, MapVecXs& zLow, MapVecXs& zUp) const
    {
        if (!optVariables_)
            throw std::runtime_error("Error in getBoundMultipliers. Optvariables not initialized");

        optVariables_->getBoundMultipliers(n, zLow, zUp);
    }

    /**
	 * @brief      Gets the values of the constraint multipliers.
	 *
	 * @param[in]  m       { The number of constraints }
	 * @param[out] lambda  The values of the constraint multipliers
	 */
    void getLambdaVars(size_t m, MapVecXs& lambda) const
    {
        if (!optVariables_)
            throw std::runtime_error("Error in getLambdaVars. Optvariables not initialized");

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
    void extractIpoptSolution(const MapConstVecXs& x,
        const MapConstVecXs& zL,
        const MapConstVecXs& zU,
        const MapConstVecXs& lambda)
    {
        if (!optVariables_)
            throw std::runtime_error("Error in extractIpoptSolution. Optvariables not initialized");

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
    void extractSnoptSolution(const MapVecXs& x,
        const MapVecXs& xMul,
        const MapVecXi& xState,
        const MapVecXs& fMul,
        const MapVecXi& fState)
    {
        if (!optVariables_)
            throw std::runtime_error("Error in extractSnoptSolution. Optvariables not initialized");

        optVariables_->setNewSnoptSolution(x, xMul, xState, fMul, fState);
    }


protected:
    std::shared_ptr<DiscreteCostEvaluatorBase<SCALAR>>
        costEvaluator_;  //! abstract base class, approximates the cost evaluation for the discrete problem
    std::shared_ptr<OptVector<SCALAR>>
        optVariables_;  //! base class, containts the optimization variables used in the NLP solvers
    std::shared_ptr<DiscreteConstraintContainerBase<SCALAR>>
        constraints_;  //! abstract base class, contains the discretized constraints for the problem
    std::shared_ptr<ct::core::DerivativesCppadJIT<-1, 1>> costCodegen_;
    std::shared_ptr<ct::core::DerivativesCppadJIT<-1, -1>> constraintsCodegen_;

    Eigen::SparseMatrix<SCALAR> hessianCost_, hessianConstraints_, hessianTotal_;

    Eigen::VectorXi iRowHessianCost_, iRowHessianConstraints_, jColHessianCost_, jColHessianConstraints_;
    Eigen::VectorXi iRowHessian_;
    Eigen::VectorXi jColHessian_;
};
}

typedef tpl::Nlp<double> Nlp;

}  // namespace optcon
}  // namespace ct
