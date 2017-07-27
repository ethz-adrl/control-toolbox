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


#ifndef CT_OPTCON_NLP_DISCRETE_COST_EVALUATOR_BASE_H_
#define CT_OPTCON_NLP_DISCRETE_COST_EVALUATOR_BASE_H_

namespace ct {
namespace optcon {
namespace tpl {

/**
 * @ingroup    NLP
 *
 * @brief      Implements an abstract base class which evaluates the cost
 *             function and its gradient in the NLP
 */
template<typename SCALAR>
class DiscreteCostEvaluatorBase{
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

} // namespace optcon
} // namespace ct

#endif //CT_OPTCON_NLP_DISCRETE_COST_EVALUATOR_BASE_HPP_
