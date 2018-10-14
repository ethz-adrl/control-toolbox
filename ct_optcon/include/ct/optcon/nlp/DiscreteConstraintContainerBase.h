/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
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

        assert(ind == c_nlp.rows());  // or throw an error
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
        std::vector<std::vector<Eigen::Triplet<SCALAR>>> tripletsConstraintTerms;
        tripletsConstraintTerms.resize(constraints_.size());

        std::vector<Eigen::SparseMatrix<SCALAR>> constraintHessians(constraints_.size());

        for (size_t c = 0; c < constraints_.size(); c++)
        {
            // get sparsity pattern of every individual constraint term
            constraints_[c]->genSparsityPatternHessian(constraints_[c]->iRowHessian(), constraints_[c]->jColHessian());
            for (size_t i = 0; i < constraints_[c]->iRowHessian().rows(); ++i)
                tripletsConstraintTerms[c].push_back(Eigen::Triplet<SCALAR>(
                    constraints_[c]->iRowHessian()(i), constraints_[c]->jColHessian()(i), SCALAR(0.1)));

            constraintHessians[c].resize(numOptVar, numOptVar);
            constraintHessians[c].setFromTriplets(tripletsConstraintTerms[c].begin(), tripletsConstraintTerms[c].end());
        }

        // sum up all hessians
        Eigen::SparseMatrix<SCALAR> constraintHessianTot(numOptVar, numOptVar);
        for (auto& n : constraintHessians)
            constraintHessianTot += n;

        iRowHessianStdVec_.clear();
        jColHessianStdVec_.clear();
        for (size_t k = 0; k < constraintHessianTot.outerSize(); ++k)
        {
            for (typename Eigen::SparseMatrix<SCALAR>::InnerIterator it(constraintHessianTot, k); it; ++it)
            {
                iRowHessianStdVec_.push_back(it.row());
                jColHessianStdVec_.push_back(it.col());
            }
        }

        iRow = Eigen::Map<Eigen::VectorXi>(iRowHessianStdVec_.data(), iRowHessianStdVec_.size(), 1);
        jCol = Eigen::Map<Eigen::VectorXi>(jColHessianStdVec_.data(), jColHessianStdVec_.size(), 1);
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
    std::vector<std::shared_ptr<DiscreteConstraintBase<SCALAR>>>
        constraints_; /*!< contains all the constraints of the NLP */

    std::vector<int> iRowHessianStdVec_;
    std::vector<int> jColHessianStdVec_;
};
}

using DiscreteConstraintContainerBase = tpl::DiscreteConstraintContainerBase<double>;
}
}
