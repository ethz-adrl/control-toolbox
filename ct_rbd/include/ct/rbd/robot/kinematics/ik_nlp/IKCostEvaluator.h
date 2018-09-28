/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {

/*!
 * @warning currently this works only with fix-base systems
 */
template <size_t NJOINTS, typename KINEMATICS>
class IKCostEvaluator : public ct::optcon::DiscreteCostEvaluatorBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IKCostEvaluator(size_t eeInd, const Eigen::Matrix3d& Qpos, const double& Qrot, const ct::rbd::RigidBodyPose& eePose)
        : costTerm_(
              new ct::rbd::TermTaskspacePoseCG<KINEMATICS, false, NJOINTS, NJOINTS, SCALAR>(eeInd, Qpos, Qrot, eePose))
    {
    }

    virtual ~IKCostEvaluator() override = default;

    virtual SCALAR eval() override
    {
        return costTerm_->evaluate(w_->getOptimizationVars(), Eigen::Matrix<double, NJOINTS, 1>::Zero(), 0.0);
    }


    virtual void evalGradient(size_t grad_length, Eigen::Map<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>>& grad) override
    {
        assert(grad_length == NJOINTS);
        grad = costTerm_->stateDerivative(w_->getOptimizationVars(), Eigen::Matrix<double, NJOINTS, 1>::Zero(), 0.0);
    }

private:
    std::shared_ptr<ct::optcon::OptVector> w_;
    std::shared_ptr<ct::rbd::TermTaskspacePoseCG<KINEMATICS, false, NJOINTS, NJOINTS, SCALAR>> costTerm_;
};

}  // namespace rbd
}  // namespace ct
