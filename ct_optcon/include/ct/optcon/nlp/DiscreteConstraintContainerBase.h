/***********************************************************************************
Copyright (c) 2016, Agile & Dexterous Robotics Lab, ETH ZURICH. All rights reserved.

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

namespace ct{
namespace optcon{

class DiscreteConstraintContainerBase
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef double Number;
	typedef Eigen::Matrix<Number, Eigen::Dynamic, 1> VectorXd;
	typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VectorXi;
	typedef Eigen::Map<VectorXd> MapVecXd;
	typedef Eigen::Map<VectorXi> MapVecXi;

	DiscreteConstraintContainerBase(){}
	// :
	// constraintsCount_(0),
	// nonZerosJacobianCount_(0)
	// {}

	virtual ~DiscreteConstraintContainerBase(){}

	virtual void prepareEvaluation() = 0;
	virtual void prepareJacobianEvaluation() = 0;

	void evalConstraints(Eigen::Map<Eigen::VectorXd>& c_nlp)
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

	void evalSparseJacobian(Eigen::Map<Eigen::VectorXd>& jac_nlp, const int nzz_jac_g)
	{
		prepareJacobianEvaluation();
		size_t ind = 0;

		for(auto constraint : constraints_)
		{
			size_t nnEle = constraint->getNumNonZerosJacobian();
			jac_nlp.segment(ind, nnEle) = constraint->evalJacobian();
			ind += nnEle;
		}

		assert(ind == (size_t) nzz_jac_g);
	}

	void getSparsityPattern(Eigen::Map<Eigen::VectorXi>& iRow_nlp, Eigen::Map<Eigen::VectorXi>& jCol_nlp, const int nnz_jac_g)
	{
		size_t rowInd = 0;
		size_t colInd = 0;
		size_t constraintCount = 0;
	
		for(auto constraint : constraints_)
		{
			size_t nnEle = constraint->getNumNonZerosJacobian();
			size_t cSize = constraint->getConstraintSize();
			Eigen::VectorXi iRow;
			Eigen::VectorXi jCol;
			iRow.resize(nnEle);
			jCol.resize(nnEle);
			constraint->genSparsityPattern(iRow, jCol);

			iRow_nlp.segment(rowInd, nnEle) = iRow.array() + constraintCount;
			jCol_nlp.segment(rowInd, nnEle) = jCol;
			constraintCount += cSize;
			rowInd += nnEle;
		}

		assert(rowInd == (size_t) nnz_jac_g);
	}

	size_t getConstraintsCount() const
	{
		size_t count = 0;
		for(auto constraint : constraints_)
			count += constraint->getConstraintSize();
		return count;
	}

	size_t getNonZerosJacobianCount() const
	{
		size_t count = 0;
		for(auto constraint : constraints_)
			count += constraint->getNumNonZerosJacobian();
		return count;
	}

	void getBounds(Eigen::Map<Eigen::VectorXd>& lowerBound, Eigen::Map<Eigen::VectorXd>& upperBound)
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

	// Eigen::VectorXd getUpperBounds() const
	// {
	// 	Eigen::VectorXd c_ub;
	// 	c_ub.resize(getConstraintsCount());
	// 	size_t ind = 0;
	// 	for(auto constraint : constraints_)
	// 	{
	// 		size_t cSize = constraint->getConstraintSize();
	// 		c_ub.segment(ind, cSize) = constraint->getUpperBound();
	// 		ind += cSize;
	// 	}

	// 	return c_ub;
	// }

	// Eigen::VectorXd getLowerBounds() const
	// {
	// 	Eigen::VectorXd c_lb;
	// 	c_lb.resize(getConstraintsCount());
	// 	size_t ind = 0;
	// 	for(auto constraint : constraints_)
	// 	{
	// 		size_t cSize = constraint->getConstraintSize();
	// 		c_lb.segment(ind, cSize) = constraint->getLowerBound();
	// 		ind += cSize;
	// 	}
		
	// 	return c_lb;	
	// }


protected:
	std::vector<std::shared_ptr<DiscreteConstraintBase>> constraints_;
	// size_t constraintsCount_;
	// size_t nonZerosJacobianCount_;


	// Eigen::VectorXd c_lb_;
	// Eigen::VectorXd c_ub_;

};

}
}


#endif //CT_OPTCON_NLP_DISCRETE_CONSTRAINT_CONTAINER_BASE_H_
