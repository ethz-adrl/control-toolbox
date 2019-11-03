/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#pragma once

#include <Eigen/Sparse>
#include <ct/core/math/DerivativesCppadJIT.h>
#include <ct/optcon/nlp/OptVector.h>
#include <ct/optcon/nlp/DiscreteConstraintContainerBase.h>
#include <ct/optcon/nlp/DiscreteCostEvaluatorBase.h>

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

    using VectorXs = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
    using VectorXi = Eigen::Matrix<int, Eigen::Dynamic, 1>;
    using MapVecXs = Eigen::Map<VectorXs>;
    using MapVecXi = Eigen::Map<VectorXi>;
    using MapConstVecXs = Eigen::Map<const VectorXs>;

    /**
     * @brief      Default constructor
     */
    Nlp() = default;
    /**
     * @brief      Destructor
     */
    virtual ~Nlp() = default;
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
        if (!costEvaluator_)
            throw std::runtime_error("Error in evaluateCostFun. Costevaluator not initialized");

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
        if (!costEvaluator_)
            throw std::runtime_error("Error in evaluateCostGradient. Costevaluator not initialized");

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
        if (!constraints_)
            throw std::runtime_error("Error in evaluateConstraints. Constraints not initialized");

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
        if (!constraints_)
            throw std::runtime_error("Error in evaluateConstraintJacobian. Constraints not initialized");

        constraints_->evalSparseJacobian(jac, nele_jac);
    }

    /**
     * @brief      Evaluates the hessian of the lagrangian
     *
     * @param[in]  nele_hes  The number of non zeros in the hessian
     * @param      hes       The values of the non-zeros of the hessian
     * @param[in]  obj_fac   The costfunction multiplier (zero or one, NLP solver uses this to query for parts of the hessian only)
     * @param      lambda    The constraint multipliers
     *
     * @note it is important to note that our NLP-solvers expect the Hessian reduced to triangular form (symmetry).
     */
    void evaluateHessian(const int nele_hes, MapVecXs& hes, const SCALAR obj_fac, MapConstVecXs& lambda)
    {
#if EIGEN_VERSION_AT_LEAST(3, 3, 0)

        // store objective-factor in matrix to ensure compatibility with other derivative-classes
        Eigen::Matrix<double, 1, 1> omega;
        omega << obj_fac;

        // evaluate Hessian values
        Eigen::VectorXd hessianCostValues, hessianConstraintsValues;

        costEvaluator_->sparseHessianValues(optVariables_->getOptimizationVars(), omega, hessianCostValues);

        hessianConstraintsValues = constraints_->sparseHessianValues(optVariables_->getOptimizationVars(), lambda);


        // collect all Hessian values (from both constraints and cost) in one vector of triplets
        // here, we re-use the the sparsity information collected in getNonZeroHessianCount()
        std::vector<Eigen::Triplet<SCALAR>, Eigen::aligned_allocator<Eigen::Triplet<SCALAR>>> triplets;
        triplets.reserve(iRowHessianCost_.rows() + iRowHessianConstraints_.rows());

        for (int i = 0; i < iRowHessianCost_.rows(); i++)
        {
            triplets.push_back(Eigen::Triplet<SCALAR>(iRowHessianCost_(i), jColHessianCost_(i), hessianCostValues(i)));
        }
        for (int i = 0; i < iRowHessianConstraints_.rows(); i++)
        {
            triplets.push_back(Eigen::Triplet<SCALAR>(
                iRowHessianConstraints_(i), jColHessianConstraints_(i), hessianConstraintsValues(i)));
        }

        // setting the sparse Hessian from Eigen-triplets automatically resolves (e.g. adds) entries at same indices
        Hessian_eval_.setFromTriplets(triplets.begin(), triplets.end());

        // IPOPT and SNOPT only expect a triangular Hessian matrix (symmetry)
        // enforce triangular view
        // todo: is there a nicer in-place conversion to triangularView?
        Hessian_eval_ = Hessian_eval_.template triangularView<Eigen::Lower>();

        hes = Eigen::Map<Eigen::VectorXd>(Hessian_eval_.valuePtr(), nele_hes, 1);

#else
        throw std::runtime_error(
            "Nlp::evaluateHessian() only available for Eigen 3.3 and newer. Please change solver settings to NOT use "
            "exact Hessians");
#endif
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
        if (!constraints_)
            throw std::runtime_error("Error in getSparsityPatternJacobian. Constraints not initialized");

        iRow.setZero();
        jCol.setZero();

        constraints_->getSparsityPattern(iRow, jCol, nele_jac);
    }

    /**
     * @brief      Gets the sparsity pattern of the Hessian of the Lagrangian
     *
     * @param[in]  nele_hes  The number of non zero elements in the hessian
     * @param      iRow      The row indices
     * @param      jCol      The column indices
     */
    void getSparsityPatternHessian(const int nele_hes, MapVecXi& iRow, MapVecXi& jCol) const
    {
        // todo this implicitly assumes that getNonZeroHessianCount() got called before. Assert this!
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
        if (!constraints_)
            throw std::runtime_error("Error in getNonZeroJacobianCount. Constraints not initialized");

        return constraints_->getNonZerosJacobianCount();
    }

    /**
     * @brief      Returns the number of non zeros in the Hessian.
     *
     * This method intelligently combines sparsity of cost and constraint Hessians using sparse Eigen operations.
     *
     * @return     The number of non zeros in the Hessian
     */
    size_t getNonZeroHessianCount()
    {
#if EIGEN_VERSION_AT_LEAST(3, 3, 0)

        // the sparse Eigen-matrices need to be resized properly, which happens in this step.
        // todo: need to assert that getNonZeroHessianCount() gets called before the first call to a Hessian evaluation.
        Hessian_eval_.resize(optVariables_->size(), optVariables_->size());
        Hessian_sparsity_.resize(optVariables_->size(), optVariables_->size());

        iRowHessianCost_.setZero();
        jColHessianCost_.setZero();
        iRowHessianConstraints_.setZero();
        jColHessianConstraints_.setZero();


        // evaluate sparsity patterns
        constraints_->getSparsityPatternHessian(
            iRowHessianConstraints_, jColHessianConstraints_, optVariables_->size());

        costEvaluator_->getSparsityPatternHessian(iRowHessianCost_, jColHessianCost_);


        // collect all Hessian sparsity values (from both constraints and cost) in one vector of triplets
        std::vector<Eigen::Triplet<SCALAR>, Eigen::aligned_allocator<Eigen::Triplet<SCALAR>>> triplets;
        triplets.reserve(iRowHessianCost_.rows() + iRowHessianConstraints_.rows());

        for (int i = 0; i < iRowHessianCost_.rows(); ++i)
            triplets.push_back(Eigen::Triplet<SCALAR>(iRowHessianCost_(i), jColHessianCost_(i), SCALAR(1.0)));

        for (int i = 0; i < iRowHessianConstraints_.rows(); ++i)
            triplets.push_back(
                Eigen::Triplet<SCALAR>(iRowHessianConstraints_(i), jColHessianConstraints_(i), SCALAR(1.0)));

        // setting the sparse Hessian from Eigen-triplets automatically resolves (e.g. adds) entries at same indices
        Hessian_sparsity_.setFromTriplets(triplets.begin(), triplets.end());

        // IPOPT and SNOPT only expect a triangular Hessian matrix (symmetry)
        // enforce triangular view
        // todo: is there a nicer in-place conversion to triangularView?
        Hessian_sparsity_ = Hessian_sparsity_.template triangularView<Eigen::Lower>();

        // lastly, collect and combine the sparsity as filled into the helper-matrix and store in Eigen::Vectors.
        std::vector<int> iRowHessianStdVec;
        std::vector<int> jColHessianStdVec;
        for (int k = 0; k < Hessian_sparsity_.outerSize(); ++k)
        {
            for (typename Eigen::SparseMatrix<SCALAR>::InnerIterator it(Hessian_sparsity_, k); it; ++it)
            {
                iRowHessianStdVec.push_back(it.row());
                jColHessianStdVec.push_back(it.col());
            }
        }
        iRowHessian_ = Eigen::Map<Eigen::VectorXi>(iRowHessianStdVec.data(), iRowHessianStdVec.size(), 1);
        jColHessian_ = Eigen::Map<Eigen::VectorXi>(jColHessianStdVec.data(), jColHessianStdVec.size(), 1);

        // the number of non-zero elements is equal to the number rows
        size_t nonZerosHessian = iRowHessian_.rows();
        return nonZerosHessian;
#else
        throw std::runtime_error(
            "Nlp::getNonZeroHessianCount() only available for Eigen 3.3 and newer. Please change solver settings to "
            "NOT use "
            "exact Hessians or upgrade Eigen version.");
        return 0;
#endif
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
    //! Ptr to cost evaluator, which approximates the cost evaluation for the discrete problem
    std::shared_ptr<DiscreteCostEvaluatorBase<SCALAR>> costEvaluator_;

    //! Ptr to optimization variable container, which holds the optimization variables used in the NLP solvers
    std::shared_ptr<OptVector<SCALAR>> optVariables_;

    //! Ptr to constraint container, which contains the discretized constraints for the problem
    std::shared_ptr<DiscreteConstraintContainerBase<SCALAR>> constraints_;

#if EIGEN_VERSION_AT_LEAST(3, 3, 0)
    Eigen::SparseMatrix<SCALAR> Hessian_eval_;
    Eigen::SparseMatrix<SCALAR> Hessian_sparsity_;  // this is just a helper data structure
#endif

    //! helper containers for calculating the sparsity patterns
    Eigen::VectorXi iRowHessianCost_, iRowHessianConstraints_, jColHessianCost_, jColHessianConstraints_;

    //! combined Hessian sparsity pattern gets stored here
    Eigen::VectorXi iRowHessian_, jColHessian_;
};
}  // namespace tpl

using Nlp = tpl::Nlp<double>;

}  // namespace optcon
}  // namespace ct
