/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich,
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <memory>
#include <Eigen/Core>
#include <ct/optcon/nlp/DiscreteConstraintBase.h>

namespace ct {
namespace optcon {
namespace tpl {

/**
 * @ingroup    NLP
 *
 * @brief      An abstract base class which serves as a container for all the
 *             discrete constraints used in the NLP
 */
template <typename SCALAR>
class DiscreteConstraintContainerBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using VectorXs = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
    using VectorXi = Eigen::Matrix<int, Eigen::Dynamic, 1>;
    using MapVecXs = Eigen::Map<VectorXs>;
    using MapVecXi = Eigen::Map<VectorXi>;

    /**
     * @brief      Default constructor
     */
    DiscreteConstraintContainerBase() = default;
    /**
     * @brief      Destructor
     */
    virtual ~DiscreteConstraintContainerBase() = default;
    /**
     * @brief      Gets called before the constraint evaluation. This method
     *             should contain all the calculations needed to evaluate the
     *             constraints
     */
    virtual void prepareEvaluation() = 0;

    /**
     * @brief      Gets called before the constraint jacobian evaluation. This
     *             method should contain all the calculations needed to evaluate
     *             the constraint jacobian
     */
    virtual void prepareJacobianEvaluation() = 0;


    /**
     * @brief      Writes the constraint evaluations into the large constraint
     *             optimization vector
     *
     * @param[out] c_nlp  The constraint vector used in the NLP
     */
    void evalConstraints(MapVecXs& c_nlp)
    {
        prepareEvaluation();
        size_t ind = 0;

        for (auto constraint : constraints_)
        {
            size_t cSize = constraint->getConstraintSize();
            c_nlp.segment(ind, cSize) = constraint->eval();
            ind += cSize;
        }

        assert(ind == static_cast<size_t>(c_nlp.rows()));  // or throw an error
    }

    void evalConstraints(VectorXs& c_nlp)
    {
        prepareEvaluation();
        size_t ind = 0;

        for (auto constraint : constraints_)
        {
            size_t cSize = constraint->getConstraintSize();
            c_nlp.segment(ind, cSize) = constraint->eval();
            ind += cSize;
        }

        assert(static_cast<int>(ind) == c_nlp.rows());  // or throw an error
    }

    /**
     * @brief      Evaluates the jacobian of the constraints and writes them
     *             into the nlp vector
     *
     * @param[out] jac_nlp    The constraint jacobian vector used in NLP
     * @param[in]  nzz_jac_g  The number of non zero elements in the jacobian
     */
    void evalSparseJacobian(MapVecXs& jac_nlp, const int nzz_jac_g)
    {
        prepareJacobianEvaluation();
        size_t ind = 0;

        for (auto constraint : constraints_)
        {
            size_t nnEle = constraint->getNumNonZerosJacobian();
            jac_nlp.segment(ind, nnEle) = constraint->evalSparseJacobian();
            ind += nnEle;
        }

        assert(static_cast<int>(ind) == nzz_jac_g);
    }

    /**
     * @brief      Retrieves the sparsity pattern of the constraint Jacobian and writes
     *             them into the nlp vectors
     *
     * @param[out] iRow_nlp   The vector containing the row indices of the non
     *                        zero entries of the constraint jacobian
     * @param[out] jCol_nlp   The vector containing the column indices of the
     *                        non zero entries of the constraint jacobian
     * @param[in]  nnz_jac_g  The number of non zero elements in the constraint jacobian
     */
    void getSparsityPattern(Eigen::Map<Eigen::VectorXi>& iRow_nlp,
        Eigen::Map<Eigen::VectorXi>& jCol_nlp,
        const int nnz_jac_g)
    {
        size_t ind = 0;
        size_t constraintCount = 0;

        for (auto constraint : constraints_)
        {
            size_t nnEle = constraint->getNumNonZerosJacobian();
            size_t cSize = constraint->getConstraintSize();
            Eigen::VectorXi iRow, jCol;
            iRow.resize(nnEle);
            jCol.resize(nnEle);
            constraint->genSparsityPattern(iRow, jCol);
            iRow_nlp.segment(ind, nnEle) = iRow.array() + constraintCount;
            jCol_nlp.segment(ind, nnEle) = jCol;
            constraintCount += cSize;
            ind += nnEle;
        }

        assert(static_cast<int>(ind) == nnz_jac_g);
    }

    /**
     * @brief      Returns the number of constraints in the NLP
     *
     * @return     The number of constraint in the NLP
     */
    size_t getConstraintsCount() const
    {
        size_t count = 0;
        for (auto constraint : constraints_)
            count += constraint->getConstraintSize();
        return count;
    }

    /**
     * @brief      Returns the number of non zeros in the constraint jacobian
     *
     * @return     The number of non zeros in the constraint jacobian
     */
    size_t getNonZerosJacobianCount() const
    {
        size_t count = 0;
        for (auto constraint : constraints_)
            count += constraint->getNumNonZerosJacobian();
        return count;
    }


    /**
     * @brief      creates the combined hessian sparsity pattern from a number of constraint terms
     *
     * @return     The number of non zeros in the constraint jacobian
     */
    void getSparsityPatternHessian(Eigen::VectorXi& iRow, Eigen::VectorXi& jCol, size_t numOptVar)
    {
#if EIGEN_VERSION_AT_LEAST(3, 3, 0)

        // important initialization
        constraintHessianTot_.resize(numOptVar, numOptVar);
        constraintHessianSparsity_.resize(numOptVar, numOptVar);

        std::vector<Eigen::Triplet<SCALAR>, Eigen::aligned_allocator<Eigen::Triplet<SCALAR>>> triplets;

        for (size_t c = 0; c < constraints_.size(); c++)
        {
            // get sparsity pattern of every individual constraint term
            constraints_[c]->genSparsityPatternHessian(constraints_[c]->iRowHessian(), constraints_[c]->jColHessian());
            for (int i = 0; i < constraints_[c]->iRowHessian().rows(); i++)
                triplets.push_back(Eigen::Triplet<SCALAR>(
                    constraints_[c]->iRowHessian()(i), constraints_[c]->jColHessian()(i), SCALAR(1.0)));
        }

        // fill in values in total constraint Hessian
        constraintHessianSparsity_.setFromTriplets(triplets.begin(), triplets.end());


        iRowHessianStdVec_.clear();
        jColHessianStdVec_.clear();
        for (int k = 0; k < constraintHessianSparsity_.outerSize(); k++)
        {
            for (typename Eigen::SparseMatrix<SCALAR>::InnerIterator it(constraintHessianSparsity_, k); it; ++it)
            {
                iRowHessianStdVec_.push_back(it.row());
                jColHessianStdVec_.push_back(it.col());
            }
        }

        iRow = Eigen::Map<Eigen::VectorXi>(iRowHessianStdVec_.data(), iRowHessianStdVec_.size(), 1);
        jCol = Eigen::Map<Eigen::VectorXi>(jColHessianStdVec_.data(), jColHessianStdVec_.size(), 1);
#else
        throw std::runtime_error(
            "getSparsityPatternHessian only available for Eigen 3.3 and newer. Please change solver settings to NOT "
            "use exact Hessians or upgrade Eigen version.");
#endif
    }


    /**
    * @brief      Evaluates the constraint Hessian
    *
    * @param[in]  optVec       The optimization variables
    * @param[in]  lambda       multipliers for Hessian matrix
    * @param[out] hes          The cost Hessian matrix coefficients
    */
    Eigen::VectorXd sparseHessianValues(const Eigen::VectorXd& optVec, const Eigen::VectorXd& lambda)
    {
#if EIGEN_VERSION_AT_LEAST(3, 3, 0)

        std::vector<Eigen::Triplet<SCALAR>, Eigen::aligned_allocator<Eigen::Triplet<SCALAR>>> triplets;

        size_t count = 0;
        for (size_t c = 0; c < constraints_.size(); c++)
        {
            // count the constraint size to hand over correct portion of multiplier vector lambda
            size_t c_nel = constraints_[c]->getConstraintSize();
            Eigen::VectorXd hessianSubValues;
            constraints_[c]->sparseHessianValues(optVec, lambda.segment(count, c_nel), hessianSubValues);
            count += c_nel;

            // add the evaluated sub-hessian elements as triplets
            for (int i = 0; i < hessianSubValues.rows(); i++)
                triplets.push_back(Eigen::Triplet<SCALAR>(
                    constraints_[c]->iRowHessian()(i), constraints_[c]->jColHessian()(i), hessianSubValues(i)));
        }

        // combine all triplets into the sparse constraint Hessian
        constraintHessianTot_.setFromTriplets(triplets.begin(), triplets.end());

        // triangular-view required (todo: need better in-place assignment)
        constraintHessianTot_ = constraintHessianTot_.template triangularView<Eigen::Lower>();

        size_t nele_constraint_hes = jColHessianStdVec_.size();
        return Eigen::VectorXd(Eigen::Map<Eigen::VectorXd>(constraintHessianTot_.valuePtr(), nele_constraint_hes, 1));
#else
        throw std::runtime_error(
            "sparseHessianValues only available for Eigen 3.3 and newer. Please use BFGS Hessian approx or upgrade "
            "Eigen version.");
        return Eigen::VectorXd::Zero();
#endif
    }

    /**
     * @brief      Retrieves the constraint bounds and writes them into the
     *             vectors used in the NLP
     *
     * @param[out]      lowerBound  The lower constraint bound
     * @param[out]      upperBound  The lower constraint bound
     */
    void getBounds(MapVecXs& lowerBound, MapVecXs& upperBound)
    {
        size_t ind = 0;
        for (auto constraint : constraints_)
        {
            size_t cSize = constraint->getConstraintSize();
            lowerBound.segment(ind, cSize) = constraint->getLowerBound();
            upperBound.segment(ind, cSize) = constraint->getUpperBound();
            ind += cSize;
        }
    }

protected:
    //!Container which holds all the constraints of the NLP
    std::vector<std::shared_ptr<DiscreteConstraintBase<SCALAR>>> constraints_;

    std::vector<int> iRowHessianStdVec_;
    std::vector<int> jColHessianStdVec_;

#if EIGEN_VERSION_AT_LEAST(3, 3, 0)
    Eigen::SparseMatrix<SCALAR> constraintHessianTot_;
    Eigen::SparseMatrix<SCALAR>
        constraintHessianSparsity_;  // helper to calculate sparsity and number of non-zero elements
#endif
};

}  // namespace tpl

using DiscreteConstraintContainerBase = tpl::DiscreteConstraintContainerBase<double>;

}  // namespace optcon
}  // namespact ct
