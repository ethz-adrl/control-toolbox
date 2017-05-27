#ifndef DMS_CONSTRAINTDISCRETIZER_HPP_
#define DMS_CONSTRAINTDISCRETIZER_HPP_

#include <ct/optcon/constraint/ConstraintContainerAD.h>
#include <ct/optcon/nlp/DiscreteConstraintBase.h>

namespace ct {
namespace optcon {


template <size_t STATE_DIM, size_t CONTROL_DIM>
class ConstraintDiscretizer : public DiscreteConstraintBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DiscreteConstraintBase BASE;
	typedef ct::core::StateVector<STATE_DIM> state_vector_t;
	typedef ct::core::StateVectorArray<STATE_DIM> state_vector_array_t;

	typedef ct::core::ControlVector<CONTROL_DIM> control_vector_t;
	typedef ct::core::ControlVectorArray<CONTROL_DIM> control_vector_array_t;

	typedef ct::core::TimeArray time_array_t;

	ConstraintDiscretizer(){}
	ConstraintDiscretizer(
		std::shared_ptr<LinearConstraintContainer<STATE_DIM, CONTROL_DIM>> c_continuous,
		std::vector<size_t> activeInd,
		size_t N)
	:
	N_(N),
	c_continuous_(c_continuous),
	activeInd_(activeInd)
	{
		continuousCount_ = c_continuous_->getConstraintCount();
		constraintsLocal_.resize(continuousCount_);
	}

	virtual void initialize(size_t c_index) override
	{
		indexTotal_ = c_index;
		nonZeroJacCount_ = c_continuous_->getConstraintJacobianNonZeroCount();
		jacLocal_.resize(nonZeroJacCount_);
		iRowLocal_.resize(nonZeroJacCount_);
		jColLocal_.resize(nonZeroJacCount_);
	}

	void updateTrajectories(
		const state_vector_array_t& stateTraj,
		const control_vector_array_t& inputTraj,
		const time_array_t& timeTraj)
	{
		assert(stateTraj.size() == N_);
		assert(inputTraj.size() == N_);
		assert(timeTraj.size() == N_);
		stateTraj_ = stateTraj;
		inputTraj_ = inputTraj;
		timeTraj_ = timeTraj;
	}

	void updateTrajectories(
		const state_vector_t& stateVec,
		const control_vector_t& inputVec,
		const double time)
	{
		stateTraj_.clear();
		inputTraj_.clear();
		timeTraj_.clear();
		stateTraj_.push_back(stateVec);
		inputTraj_.push_back(inputVec);
		timeTraj_.push_back(time);
	}

	virtual size_t getEvaluation(Eigen::Map<Eigen::VectorXd>& val, size_t count) override
	{
		constraintsLocal_.setZero();
		size_t countLocalTot = count;
		size_t countLocal = 0;

		for(auto ind : activeInd_)
		{			
			c_continuous_->setTimeStateInput(timeTraj_[ind], stateTraj_[ind], inputTraj_[ind]);
			c_continuous_->evaluate(constraintsLocal_, countLocal);
			val.segment(countLocalTot, countLocal) = constraintsLocal_;
			countLocalTot += countLocal;
		}
		return countLocalTot;
	}

	virtual size_t evalConstraintJacobian(Eigen::Map<Eigen::VectorXd>& val, size_t count) override
	{
		jacLocal_.setZero();
		size_t countLocalTot = count;
		size_t countLocal = 0;

		for(auto ind : activeInd_)
		{
			c_continuous_->setTimeStateInput(timeTraj_[ind], stateTraj_[ind], inputTraj_[ind]);
			c_continuous_->evalJacSparse(jacLocal_, countLocal);
			val.segment(countLocalTot, countLocal)  = jacLocal_;
			countLocalTot += countLocal;
		}

		return countLocalTot;		
	}

	virtual size_t getNumNonZerosJacobian() override
	{
		return activeInd_.size() * nonZeroJacCount_;
	}

	virtual size_t genSparsityPattern(
		Eigen::Map<Eigen::VectorXi>& iRow_vec,
		Eigen::Map<Eigen::VectorXi>& jCol_vec,
		size_t indexNumber) override
	{
		size_t countLocal = indexNumber;
		size_t i = 0;

		for(auto ind : activeInd_)
		{
			indexNumber += c_continuous_->generateSparsityPatternJacobian(iRowLocal_, jColLocal_);
			iRow_vec.segment(countLocal, nonZeroJacCount_) = iRowLocal_.array() + BASE::indexTotal_ + i * continuousCount_;
			jCol_vec.segment(countLocal, nonZeroJacCount_) = jColLocal_.array() + ind * (STATE_DIM + CONTROL_DIM);
			countLocal = indexNumber;
			i++;
		}

		return indexNumber;
	}

	virtual void getLowerBound(Eigen::VectorXd& c_lb) override
	{
		constraintsLocal_.setZero();
		size_t countLocalTot = BASE::indexTotal_;
		size_t countLocal = 0;

		for(size_t i = 0; i < activeInd_.size(); ++i)
		{
			c_continuous_->getLowerBound(constraintsLocal_, countLocal);
			c_lb.segment(countLocalTot, countLocal) = constraintsLocal_;
			countLocalTot += countLocal;
		}
	}

	virtual void getUpperBound(Eigen::VectorXd& c_ub) override
	{
		constraintsLocal_.setZero();
		size_t countLocalTot = BASE::indexTotal_;
		size_t countLocal = 0;

		for(size_t i = 0; i < activeInd_.size(); ++i)
		{
			c_continuous_->getUpperBound(constraintsLocal_, countLocal);
			c_ub.segment(countLocalTot, countLocal) = constraintsLocal_;
			countLocalTot += countLocal;
		}
	}

	virtual size_t getConstraintSize() override
	{
		size_t discreteCount = 0;
		for(size_t i = 0; i < activeInd_.size(); ++i)
			discreteCount += continuousCount_;

		return discreteCount; 
	}


private:
	size_t N_;
	std::shared_ptr<LinearConstraintContainer<STATE_DIM, CONTROL_DIM>> c_continuous_;
	std::vector<size_t> activeInd_;
	
	size_t continuousCount_;
	Eigen::VectorXd constraintsLocal_;
	Eigen::VectorXd jacLocal_;
	Eigen::VectorXi iRowLocal_;
	Eigen::VectorXi jColLocal_;

	// Trajectories, along which the constraints will be discretized 
	state_vector_array_t stateTraj_;
	control_vector_array_t inputTraj_;
	time_array_t timeTraj_;
	size_t nonZeroJacCount_;

};



}
}


#endif //DMS_CONSTRAINTDISCRETIZER_HPP_
