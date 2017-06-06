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


#ifndef CT_OPTCON_DMS_CORE_CONSTRAINTS_TIME_HORIZON_EQUALITY_CONSTRAINT_H_
#define CT_OPTCON_DMS_CORE_CONSTRAINTS_TIME_HORIZON_EQUALITY_CONSTRAINT_H_

#include <ct/optcon/nlp/DiscreteConstraintBase.h>

namespace ct {
namespace optcon {

/**
 * @ingroup    DMS
 *
 * @brief      This is the implementation of the time horizon constraint when
 *             using time grid optimization.
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The input dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM>
class TimeHorizonEqualityConstraint : public DiscreteConstraintBase
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DiscreteConstraintBase BASE;

	/**
	 * @brief      Default constructor
	 */
	TimeHorizonEqualityConstraint(){}

	/**
	 * @brief      Custom constructor
	 *
	 * @param[in]  w         The optimization variables
	 * @param[in]  timeGrid  The dms time grid
	 * @param[in]  settings  The dms settings
	 */
	TimeHorizonEqualityConstraint(
		std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w,
		std::shared_ptr<TimeGrid> timeGrid,
		DmsSettings settings
		):
		w_(w),
		timeGrid_(timeGrid),
		settings_(settings)
	{
		// lower bound is number of shot times the lower bound for each interval h
		lb_ << (settings_.N_ * settings_.h_min_ - settings_.T_);
		ub_ << 0.0;

		std::cout << " ... time horizon lower bound: " << settings_.T_ + lb_ << std::endl;
		std::cout << " ... time horizon upper bound: " << settings_.T_ + ub_ << std::endl;
	}

	virtual Eigen::VectorXd eval() override
	{
		Eigen::Matrix<double, 1, 1> mat;
		mat << (timeGrid_->getOptimizedTimeHorizon() - settings_.T_);
		return mat;
	}	

	virtual Eigen::VectorXd evalSparseJacobian() override
	{
		Eigen::VectorXd one(settings_.N_);
		one.setConstant(1.0);
		return one;
	}

	virtual size_t getNumNonZerosJacobian() override
	{
		return settings_.N_;
	}

	virtual void genSparsityPattern(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override
	{
		for(size_t i = 0; i < settings_.N_; ++i)
		{
			iRow_vec(i) = 0;
			jCol_vec(i) = w_->getTimeSegmentIndex(i);
		}
	}

	virtual Eigen::VectorXd getLowerBound() override
	{
		return lb_;
	}

	virtual Eigen::VectorXd getUpperBound() override
	{
		return ub_;
	}

	virtual size_t getConstraintSize() override
	{
		return 1;
	}

private:
	std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w_;
	std::shared_ptr<TimeGrid> timeGrid_;
	DmsSettings settings_;

	//Constraint bounds
	Eigen::Matrix<double, 1, 1> lb_;
	Eigen::Matrix<double, 1, 1> ub_;
};

} // namespace optcon
} // namespace ct

#endif //CT_OPTCON_DMS_CORE_CONSTRAINTS_TIME_HORIZON_EQUALITY_CONSTRAINT_H_

