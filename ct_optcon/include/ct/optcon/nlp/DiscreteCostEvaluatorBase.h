/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
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
};
}

using DiscreteCostEvaluatorBase = tpl::DiscreteCostEvaluatorBase<double>;

}  // namespace optcon
}  // namespace ct
