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
		std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w,
		std::shared_ptr<LinearConstraintContainer<STATE_DIM, CONTROL_DIM>> c_continuous,
		std::vector<size_t> activeInd)
	:
	w_(w),
	c_continuous_(c_continuous),
	activeInd_(activeInd)
	{
		continuousCount_ = c_continuous_->getConstraintCount();
		constraintsLocal_.resize(continuousCount_);
		discreteConstraints_.resize(activeInd_.size() * continuousCount_);
		discreteLowerBound_.resize(activeInd_.size() * continuousCount_);
		discreteUpperBound_.resize(activeInd_.size() * continuousCount_);

		nonZeroJacCount_ = c_continuous_->getConstraintJacobianNonZeroCount();
		jacLocal_.resize(nonZeroJacCount_);
		iRowLocal_.resize(nonZeroJacCount_);
		jColLocal_.resize(nonZeroJacCount_);
		discreteJac_.resize(activeInd_.size() * nonZeroJacCount_);
		discreteIRow_.resize(activeInd_.size() * nonZeroJacCount_);
		discreteJCol_.resize(activeInd_.size() * nonZeroJacCount_);
	}

	// virtual void initialize(size_t c_index) override
	// {
	// 	indexTotal_ = c_index;
	// 	nonZeroJacCount_ = c_continuous_->getConstraintJacobianNonZeroCount();
	// 	jacLocal_.resize(nonZeroJacCount_);
	// 	iRowLocal_.resize(nonZeroJacCount_);
	// 	jColLocal_.resize(nonZeroJacCount_);
	// 	discreteJac_.resize(activeInd_.size() * nonZeroJacCount_);
	// 	discreteIRow_.resize(activeInd_.size() * nonZeroJacCount_);
	// 	discreteJCol_.resize(activeInd_.size() * nonZeroJacCount_);
	// }

	void updateTrajectories(
		const state_vector_array_t& stateTraj,
		const control_vector_array_t& inputTraj,
		const time_array_t& timeTraj)
	{
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

	virtual Eigen::VectorXd eval() override
	{
		constraintsLocal_.setZero();
		size_t constraintSize = 0;
		size_t discreteInd = 0;

		for(auto ind : activeInd_)
		{			
			c_continuous_->setTimeStateInput(timeTraj_[ind], stateTraj_[ind], inputTraj_[ind]);
			c_continuous_->evaluate(constraintsLocal_, constraintSize);
			discreteConstraints_.segment(discreteInd, constraintSize) = constraintsLocal_;
			discreteInd += constraintSize;
		}

		return discreteConstraints_;
	}

	virtual Eigen::VectorXd evalSparseJacobian() override
	{
		jacLocal_.setZero();
		size_t jacSize = 0;
		size_t discreteInd = 0;

		for(auto ind : activeInd_)
		{
			c_continuous_->setTimeStateInput(timeTraj_[ind], stateTraj_[ind], inputTraj_[ind]);
			c_continuous_->evalJacSparse(jacLocal_, jacSize);
			discreteJac_.segment(discreteInd, jacSize) = jacLocal_;
			discreteInd += jacSize;
		}

		return discreteJac_;		
	}

	virtual size_t getNumNonZerosJacobian() override
	{
		// std::cout << "calling getNumNonZerosJacobian " << activeInd_.size() * nonZeroJacCount_ << std::endl;
		return activeInd_.size() * nonZeroJacCount_;
	}

	virtual void genSparsityPattern(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override
	{
		size_t discreteInd = 0;
		size_t nnEle = 0;
		size_t i = 0;

		for(auto ind : activeInd_)
		{
			nnEle = c_continuous_->generateSparsityPatternJacobian(iRowLocal_, jColLocal_);
			discreteIRow_.segment(discreteInd, nnEle) = iRowLocal_.array() + i * continuousCount_;
			discreteJCol_.segment(discreteInd, nnEle) = jColLocal_.array() + w_->getStateIndex(ind);
			discreteInd += nnEle;
			i++;
		}
		iRow_vec = discreteIRow_;
		jCol_vec = discreteJCol_;
	}

	virtual Eigen::VectorXd getLowerBound() override
	{
		constraintsLocal_.setZero();
		size_t discreteInd = 0;
		size_t constraintSize = 0;

		for(size_t i = 0; i < activeInd_.size(); ++i)
		{
			c_continuous_->getLowerBound(constraintsLocal_, constraintSize);
			discreteLowerBound_.segment(discreteInd, constraintSize) = constraintsLocal_;
			discreteInd += constraintSize;
		}
		return discreteLowerBound_;
	}

	virtual Eigen::VectorXd getUpperBound() override
	{
		constraintsLocal_.setZero();
		size_t discreteInd = 0;
		size_t constraintSize = 0;

		for(size_t i = 0; i < activeInd_.size(); ++i)
		{
			c_continuous_->getUpperBound(constraintsLocal_, constraintSize);
			discreteUpperBound_.segment(discreteInd, constraintSize) = constraintsLocal_;
			discreteInd += constraintSize;
		}
		return discreteUpperBound_;
	}

	virtual size_t getConstraintSize() override
	{
		size_t discreteCount = 0;
		for(size_t i = 0; i < activeInd_.size(); ++i)
			discreteCount += continuousCount_;


		// std::cout << "calling getConstraintSize" << discreteCount << std::endl;
		return discreteCount; 
	}


private:
	std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w_;
	std::shared_ptr<LinearConstraintContainer<STATE_DIM, CONTROL_DIM>> c_continuous_;
	std::vector<size_t> activeInd_;
	
	size_t continuousCount_;
	Eigen::VectorXd constraintsLocal_;
	Eigen::VectorXd discreteConstraints_;
	Eigen::VectorXd discreteLowerBound_;
	Eigen::VectorXd discreteUpperBound_;

	Eigen::VectorXd jacLocal_;
	Eigen::VectorXd discreteJac_;
	Eigen::VectorXi iRowLocal_;
	Eigen::VectorXi discreteIRow_;
	Eigen::VectorXi jColLocal_;
	Eigen::VectorXi discreteJCol_;

	// Trajectories, along which the constraints will be discretized 
	state_vector_array_t stateTraj_;
	control_vector_array_t inputTraj_;
	time_array_t timeTraj_;
	size_t nonZeroJacCount_;

};



}
}


#endif //DMS_CONSTRAINTDISCRETIZER_HPP_
