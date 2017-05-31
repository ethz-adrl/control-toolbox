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

	DiscreteConstraintContainerBase()
	:
	constraintsCount_(0),
	nonZerosJacobianCount_(0)
	{}

	virtual ~DiscreteConstraintContainerBase(){}

	virtual void prepareEvaluation() = 0;

	void evalConstraints(Eigen::Map<Eigen::VectorXd>& c_nlp)
	{
		prepareEvaluation();
		size_t count = 0;

		for(auto constraint : constraints_)
			count = constraint->getEvaluation(c_nlp, count);

		assert(count == c_nlp.rows()); // or throw an error
	}

	void getSparsityPattern(Eigen::Map<Eigen::VectorXi>& iRow_vec, Eigen::Map<Eigen::VectorXi>& jCol_vec, const int nnz_jac_g)
	{
		size_t count = 0;
	
		for(auto constraint : constraints_)
			count = constraint->genSparsityPattern(iRow_vec, jCol_vec, count);	

		assert(count == (size_t) nnz_jac_g);
	}

	virtual void prepareJacobianEvaluation() = 0;

	void evalSparseJacobian(Eigen::Map<Eigen::VectorXd>& val, const int nzz_jac_g)
	{
		size_t count = 0;

		for(auto constraint : constraints_)
			count = constraint->evalConstraintJacobian(val, count);	

		assert(count == (size_t) nzz_jac_g);
	}

	const size_t getConstraintsCount() const
	{
		return constraintsCount_;
	}

	const size_t getNonZerosJacobianCount() const
	{
		return nonZerosJacobianCount_;
	}

	const Eigen::VectorXd getUpperBounds() const
	{
		return c_ub_;
	}

	const Eigen::VectorXd getLowerBounds() const
	{
		return c_lb_;
	}


protected:
	size_t constraintsCount_;
	size_t nonZerosJacobianCount_;

	std::vector<std::shared_ptr<DiscreteConstraintBase>> constraints_;

	Eigen::VectorXd c_lb_;
	Eigen::VectorXd c_ub_;

};

}
}


#endif //CT_OPTCON_NLP_DISCRETE_CONSTRAINT_CONTAINER_BASE_H_
