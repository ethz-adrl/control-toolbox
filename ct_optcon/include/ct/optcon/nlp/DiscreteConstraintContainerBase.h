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

#ifndef CT_OPTCON_NLP_DISCRETE_CONSTRAINT_CONTAINER_BASE_H_
#define CT_OPTCON_NLP_DISCRETE_CONSTRAINT_CONTAINER_BASE_H_


#include "DiscreteConstraintBase.h"

namespace ct {
namespace optcon {
namespace tpl {

/**
 * @ingroup    NLP
 *
 * @brief      An abstract base class which serves as a container for all the
 *             discrete constraints used in the NLP
 */
template<typename SCALAR>
class DiscreteConstraintContainerBase
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
	typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VectorXi;
	typedef Eigen::Map<VectorXs> MapVecXs;
	typedef Eigen::Map<VectorXi> MapVecXi;

	/**
	 * @brief      Default constructor
	 */
	DiscreteConstraintContainerBase(){}

	/**
	 * @brief      Destructor
	 */
	virtual ~DiscreteConstraintContainerBase(){}

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

		for(auto constraint : constraints_)
		{
			size_t cSize = constraint->getConstraintSize();
			c_nlp.segment(ind, cSize) = constraint->eval();
			ind += cSize;
		}

		assert(ind == c_nlp.rows()); // or throw an error
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

		for(auto constraint : constraints_)
		{
			size_t nnEle = constraint->getNumNonZerosJacobian();
			jac_nlp.segment(ind, nnEle) = constraint->evalSparseJacobian();
			ind += nnEle;
		}

		assert(ind == (size_t) nzz_jac_g);
	}

	/**
	 * @brief      Retrieves the sparsity pattern of the constraints and writes
	 *             them into the nlp vectors
	 *
	 * @param[out] iRow_nlp   The vector containing the row indices of the non
	 *                        zero entries of the constraint jacobian
	 * @param[out] jCol_nlp   The vector containing the column indices of the
	 *                        non zero entries of the constraint jacobian
	 * @param[in]  nnz_jac_g  The number of non zero elements in the constraint jacobian
	 */
	void getSparsityPattern(Eigen::Map<Eigen::VectorXi>& iRow_nlp, Eigen::Map<Eigen::VectorXi>& jCol_nlp, const int nnz_jac_g)
	{
		size_t ind = 0;
		size_t constraintCount = 0;
	
		for(auto constraint : constraints_)
		{
			size_t nnEle = constraint->getNumNonZerosJacobian();
			size_t cSize = constraint->getConstraintSize();
			Eigen::VectorXi iRow, jCol;
			iRow.resize(nnEle); jCol.resize(nnEle);
			constraint->genSparsityPattern(iRow, jCol);
			iRow_nlp.segment(ind, nnEle) = iRow.array() + constraintCount;
			jCol_nlp.segment(ind, nnEle) = jCol;
			constraintCount += cSize;
			ind += nnEle;
		}

		assert(ind == (size_t) nnz_jac_g);
	}

	/**
	 * @brief      Returns the number of constraints in the NLP
	 *
	 * @return     The number of constraint in the NLP
	 */
	size_t getConstraintsCount() const
	{
		size_t count = 0;
		for(auto constraint : constraints_)
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
		for(auto constraint : constraints_)
			count += constraint->getNumNonZerosJacobian();
		return count;
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
		for(auto constraint : constraints_)
		{
			size_t cSize = constraint->getConstraintSize();
			lowerBound.segment(ind, cSize) = constraint->getLowerBound();
			upperBound.segment(ind, cSize) = constraint->getUpperBound();
			ind += cSize;
		}		
	}

protected:
	std::vector<std::shared_ptr<DiscreteConstraintBase<SCALAR>>> constraints_; /*!< contains all the constraints of the NLP */

};

}

typedef	tpl::DiscreteConstraintContainerBase<double> DiscreteConstraintContainerBase;

}
}


#endif //CT_OPTCON_NLP_DISCRETE_CONSTRAINT_CONTAINER_BASE_H_
