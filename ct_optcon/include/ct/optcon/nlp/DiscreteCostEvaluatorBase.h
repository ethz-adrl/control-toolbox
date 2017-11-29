/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#pragma once

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
    DiscreteCostEvaluatorBase(){};

    /**
   * @brief     Destructor.
   */
    virtual ~DiscreteCostEvaluatorBase(){};


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

typedef tpl::DiscreteCostEvaluatorBase<double> DiscreteCostEvaluatorBase;

}  // namespace optcon
}  // namespace ct
