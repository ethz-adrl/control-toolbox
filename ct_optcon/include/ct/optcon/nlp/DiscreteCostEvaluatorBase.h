/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#pragma once

#include <Eigen/Core>

namespace ct {
namespace optcon {
namespace tpl {

/**
 * @ingroup    NLP
 *
 * @brief      Implements an abstract base class which evaluates the cost
 *             function and its gradient in the NLP
 */
template <typename SCALAR>
class DiscreteCostEvaluatorBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
    * @brief      Default constructor
    */
    DiscreteCostEvaluatorBase() = default;

    /**
   * @brief     Destructor.
   */
    virtual ~DiscreteCostEvaluatorBase() = default;


    /**
   * @brief      Evaluates the cost function
   *
   * @return     The evaluates cost function
   */
    virtual SCALAR eval() = 0;

    /**
    * @brief      Evaluates the cost gradient
    *
    * @param[in]  grad_length  The size of the gradient vector
    * @param[out] grad         The values of the gradient
    */
    virtual void evalGradient(size_t grad_length, Eigen::Map<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>>& grad) = 0;

    virtual void getSparsityPatternHessian(Eigen::VectorXi& iRow, Eigen::VectorXi& jCol)
    {
        throw std::runtime_error(
            "Hessian evaluation not implemented for this cost function. Use limited-memory Hessian approximation!");
    }

    /**
    * @brief      Evaluates the cost hessian
    *
    * @param[in]  optVec       The optimization variables
    * @param[in]  lambda       multipliers for hessian matrix
    * @param[out] hes          The cost hessian matrix coeff
    */
    virtual void sparseHessianValues(const Eigen::VectorXd& optVec, const Eigen::VectorXd& lambda, Eigen::VectorXd& hes)
    {
        throw std::runtime_error(
            "Hessian evaluation not implemented for this cost function. Use limited-memory Hessian approximation!");
    }
};
}

using DiscreteCostEvaluatorBase = tpl::DiscreteCostEvaluatorBase<double>;

}  // namespace optcon
}  // namespace ct
