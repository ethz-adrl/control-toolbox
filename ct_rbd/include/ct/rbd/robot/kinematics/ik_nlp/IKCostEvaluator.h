/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/robot/Kinematics.h>
#include <ct/rbd/state/RBDState.h>
#include "IKRegularizerBase.h"

namespace ct {
namespace rbd {


/*!
 * @brief Inverse Kinematics cost evaluator for NLP
 * @warning currently this works only with fix-base systems
 */
template <typename KINEMATICS_AD, typename SCALAR = double>
class IKCostEvaluator final : public ct::optcon::tpl::DiscreteCostEvaluatorBase<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const size_t NJOINTS = KINEMATICS_AD::NJOINTS;

    IKCostEvaluator(
    		size_t eeInd,
			const Eigen::Matrix3d& Qpos = Eigen::Matrix3d::Identity(),
			const double& Qrot = 1.0)
        : w_(nullptr),
		  costTerm_(
              new ct::rbd::TermTaskspacePoseCG<KINEMATICS_AD, false, NJOINTS, NJOINTS>(eeInd, Qpos, Qrot)),
			  ikRegularizer_(nullptr)
    {
    }

    IKCostEvaluator(const std::string& costFunctionPath, const std::string& termName, bool verbose)
        : w_(nullptr),
		  costTerm_(
              new ct::rbd::TermTaskspacePoseCG<KINEMATICS_AD, false, NJOINTS, NJOINTS>(costFunctionPath, termName, verbose)),
			  ikRegularizer_(nullptr)
    {
    }

    virtual ~IKCostEvaluator() override = default;

    //! opt vector needs to be set by NLP solver
    void setOptVector(std::shared_ptr<ct::optcon::tpl::OptVector<SCALAR>> optVector)
    {
    	w_ = optVector;
    }

    // set an optional regularizer (if necessary)
    void setRegularizer(std::shared_ptr<IKRegularizerBase> reg) { ikRegularizer_ = reg; }

    ct::rbd::RigidBodyPose getTargetPose()
    {
    	return costTerm_->getReferencePose();
    }

    // set the target pose as rbd pose directly
    void setTargetPose( const ct::rbd::RigidBodyPose& rbdPose)
    {
		costTerm_->setReferencePosition(rbdPose.position().toImplementation());
		costTerm_->setReferenceOrientation(rbdPose.getRotationQuaternion().toImplementation());
    }

    // set the target pose as separate position and quaternion
    void setTargetPose(const core::StateVector<3>& w_pos_des, const Eigen::Quaterniond& w_q_des)
    {
		costTerm_->setReferencePosition(w_pos_des);
		costTerm_->setReferenceOrientation(w_q_des);
	}

    // set the gart pose as euler angles directly
    void setTargetPose(const core::StateVector<3>& w_pos_des, const Eigen::Matrix3d& eulerXyz)
    {
    	costTerm_->setReferencePosition(w_pos_des);
    	costTerm_->setReferenceOrientation(Eigen::Quaterniond(Eigen::AngleAxisd(eulerXyz(0), Eigen::Vector3d::UnitX()) *
                        Eigen::AngleAxisd(eulerXyz(1), Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(eulerXyz(2), Eigen::Vector3d::UnitZ())));
    }

    //! evaluate cost function
    virtual SCALAR eval() override
    {
    	if(w_ != nullptr)
    		return costTerm_->evaluate(w_->getOptimizationVars(), Eigen::Matrix<double, NJOINTS, 1>::Zero(), 0.0);
    	else
    		throw std::runtime_error("IKCostEvaluator: optimization vector not set.");
    }

    //! retrieve first-order derivative
    virtual void evalGradient(size_t grad_length, Eigen::Map<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>>& grad) override
    {
        assert(grad_length == NJOINTS);
     	if(w_ != nullptr)
            grad = costTerm_->stateDerivative(w_->getOptimizationVars(), Eigen::Matrix<double, NJOINTS, 1>::Zero(), 0.0);
    	else
    		throw std::runtime_error("IKCostEvaluator: optimization vector not set.");
    }


    // todo make lower triangular (when convenient)
    //! retrieve second order derivative
    void sparseHessianValues(const Eigen::VectorXd& jointAngles, const Eigen::VectorXd& obj_fac, Eigen::VectorXd& hes) override
    {
        hes.resize(NJOINTS * NJOINTS);

     	if(w_ != nullptr)
     	{
     		// map hessian value-vector to matrix
     		Eigen::Map<Eigen::Matrix<SCALAR, NJOINTS, NJOINTS>> Hmat (hes.data(), NJOINTS, NJOINTS);
     		Hmat = obj_fac(0,0) * costTerm_->stateSecondDerivative(w_->getOptimizationVars(), Eigen::Matrix<double, NJOINTS, 1>::Zero(), 0.0);

     		if(ikRegularizer_)
     			Hmat += ikRegularizer_->computeRegularizer(w_->getOptimizationVars());
     	}
        else
    		throw std::runtime_error("IKCostEvaluator: optimization vector not set.");
    }

    // todo make lower triangular (when convenient)
    //! create sparsity pattern for the hessian
    virtual void getSparsityPatternHessian(Eigen::VectorXi& iRow,
        Eigen::VectorXi& jCol) override
    {
        iRow.resize(NJOINTS * NJOINTS);
        jCol.resize(NJOINTS * NJOINTS);

    	size_t count = 0;
    	for(size_t i = 0; i<NJOINTS; i++)
    	{
    		for (size_t j = 0; j<NJOINTS; j++)
    		{
				iRow(count) = i;
    			jCol(count) = j;
    			count++;
    		}
    	}
    }


private:
    std::shared_ptr<ct::optcon::tpl::OptVector<SCALAR>> w_;

    std::shared_ptr<ct::rbd::TermTaskspacePoseCG<KINEMATICS_AD, false, NJOINTS, NJOINTS>> costTerm_;

    std::shared_ptr<IKRegularizerBase> ikRegularizer_;
};

}  // namespace rbd
}  // namespace ct
