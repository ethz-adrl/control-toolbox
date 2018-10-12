/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/robot/Kinematics.h>
#include <ct/rbd/state/RBDState.h>

namespace ct {
namespace rbd {

/*!
 * @brief Inverse Kinematics cost evaluator
 * @warning currently this works only with fix-base systems
 */
template <typename KINEMATICS, typename SCALAR = double>
class IKCostEvaluator : public ct::optcon::tpl::DiscreteCostEvaluatorBase<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IKCostEvaluator(
    		size_t eeInd,
			const Eigen::Matrix3d& Qpos = Eigen::Matrix3d::Identity(),
			const double& Qrot = 1.0)
        : w_(nullptr),
		  costTerm_(
              new ct::rbd::TermTaskspacePoseCG<KINEMATICS, false, KINEMATICS::NJOINTS, KINEMATICS::NJOINTS>(eeInd, Qpos, Qrot))
    {
    }

    IKCostEvaluator(const std::string& costFunctionPath, const std::string& termName, bool verbose)
        : w_(nullptr),
		  costTerm_(
              new ct::rbd::TermTaskspacePoseCG<KINEMATICS, false, KINEMATICS::NJOINTS, KINEMATICS::NJOINTS>(costFunctionPath, termName, verbose))
    {
    }

    virtual ~IKCostEvaluator() override = default;

    //! needs to be set by NLP solver
    void setOptVector(std::shared_ptr<ct::optcon::tpl::OptVector<SCALAR>> optVector)
    {
    	w_ = optVector;
    }


    ct::rbd::RigidBodyPose getTargetPose()
    {
    	return costTerm_->getReferencePose();
    }

    // set the target pose as rbd pose directly
    void setTargetPose( const ct::rbd::RigidBodyPose& rbdPose)
    {
		costTerm_->setReferencePosition(rbdPose.position().toImplementation());
		costTerm_->setReferenceOrientation(rbdPose.getRotationQuaternion().toImplementation());
		costTerm_->setup(); // TODO: check if this is really required
    }

    // set the target pose as separate position and quaternion
    void setTargetPose(const core::StateVector<3>& w_pos_des, const Eigen::Quaterniond& w_q_des)
    {
		costTerm_->setReferencePosition(w_pos_des);
		costTerm_->setReferenceOrientation(w_q_des);
		costTerm_->setup(); // TODO: check if this is really required
	}

    // set the gart pose as euler angles directly
    void setTargetPose(const core::StateVector<3>& w_pos_des, const Eigen::Matrix3d& eulerXyz)
    {
    	costTerm_->setReferencePosition(w_pos_des);
    	costTerm_->setReferenceOrientation(Eigen::Quaterniond(Eigen::AngleAxisd(eulerXyz(0), Eigen::Vector3d::UnitX()) *
                        Eigen::AngleAxisd(eulerXyz(1), Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(eulerXyz(2), Eigen::Vector3d::UnitZ())));
		costTerm_->setup(); // TODO: check if this is really required
    }

    //! evaluate cost function
    virtual SCALAR eval() override
    {
    	if(w_ != nullptr)
    		return costTerm_->evaluate(w_->getOptimizationVars(), Eigen::Matrix<double, KINEMATICS::NJOINTS, 1>::Zero(), 0.0);
    	else
    		throw std::runtime_error("IKCostEvaluator: optimization vector not set.");
    }

    //! retrieve first-order derivative
    virtual void evalGradient(size_t grad_length, Eigen::Map<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>>& grad) override
    {
        assert(grad_length == KINEMATICS::NJOINTS);
     	if(w_ != nullptr)
            grad = costTerm_->stateDerivative(w_->getOptimizationVars(), Eigen::Matrix<double, KINEMATICS::NJOINTS, 1>::Zero(), 0.0);
    	else
    		throw std::runtime_error("IKCostEvaluator: optimization vector not set.");
    }

    //! retrieve second order derivative
    virtual void evalHessian(const int num_el, Eigen::Map<Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>>& hes) override
    {
        assert(num_el == KINEMATICS::NJOINTS*KINEMATICS::NJOINTS);
     	if(w_ != nullptr)
            hes = costTerm_->stateSecondDerivative(w_->getOptimizationVars(), Eigen::Matrix<double, KINEMATICS::NJOINTS, 1>::Zero(), 0.0);
    	else
    		throw std::runtime_error("IKCostEvaluator: optimization vector not set.");
    };

    // create sparsity pattern for the hessian
    virtual void getSparsityPatternHessian(Eigen::Map<Eigen::VectorXi>& iRow,
        Eigen::Map<Eigen::VectorXi>& jCol,
        const int nnz_hes)
    {
    	size_t count = 0;
    	for(size_t i = 0; i<KINEMATICS::NJOINTS; i++)
    	{
    		iRow(count) = i;
    		for (size_t j = 0; j<KINEMATICS::NJOINTS; j++)
    		{
    			jCol(count) = j;
    			count++;
    		}
    	}
    	assert(count == nnz_hes);
    }

    virtual size_t getNonZeroHessianCount() override
    {
    	return KINEMATICS::NJOINTS * KINEMATICS::NJOINTS;
    }

private:
    std::shared_ptr<ct::optcon::tpl::OptVector<SCALAR>> w_;
    std::shared_ptr<ct::rbd::TermTaskspacePoseCG<KINEMATICS, false, KINEMATICS::NJOINTS, KINEMATICS::NJOINTS>> costTerm_;
};

}  // namespace rbd
}  // namespace ct
