/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
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
template <typename KINEMATICS, typename SCALAR = double>
class IKCostEvaluator final : public ct::optcon::tpl::DiscreteCostEvaluatorBase<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t NJOINTS = KINEMATICS::NJOINTS;

    IKCostEvaluator(size_t eeInd,
        const Eigen::Matrix3d& Qpos = Eigen::Matrix3d::Identity(),
        const Eigen::Matrix3d& Qrot = Eigen::Matrix3d::Identity())
        : w_(nullptr),
          goalCostTerm_(new ct::rbd::TermTaskspaceGeometricJacobian<KINEMATICS, NJOINTS, NJOINTS>(eeInd, Qpos, Qrot)),
          jointRefTerm_(nullptr),
          ikRegularizer_(nullptr)
    {
    }

    IKCostEvaluator(const std::string& costFunctionPath,
        const std::string& termTaskspaceName,
        const bool verbose = false)
        : w_(nullptr),
          goalCostTerm_(new ct::rbd::TermTaskspaceGeometricJacobian<KINEMATICS, NJOINTS, NJOINTS>(costFunctionPath,
              termTaskspaceName,
              verbose)),
          jointRefTerm_(nullptr),
          ikRegularizer_(nullptr)
    {
    }

    // additional constructor loading a joint position reference term
    IKCostEvaluator(const std::string& costFunctionPath,
        const std::string& termTaskspaceName,
        const std::string& termJointPosName,
        const bool verbose = false)
        : w_(nullptr),
          goalCostTerm_(new ct::rbd::TermTaskspaceGeometricJacobian<KINEMATICS, NJOINTS, NJOINTS>(costFunctionPath,
              termTaskspaceName,
              verbose)),
          jointRefTerm_(
              new ct::optcon::TermQuadratic<NJOINTS, NJOINTS, SCALAR>(costFunctionPath, termJointPosName, verbose)),
          ikRegularizer_(nullptr)
    {
    }

    ~IKCostEvaluator() override = default;

    //! opt vector needs to be set by NLP solver
    void setOptVector(std::shared_ptr<ct::optcon::tpl::OptVector<SCALAR>> optVector) { w_ = optVector; }

    // set an optional regularizer (if necessary)
    void setRegularizer(std::shared_ptr<IKRegularizerBase> reg) { ikRegularizer_ = reg; }

    ct::rbd::RigidBodyPose getTargetPose() { return goalCostTerm_->getReferencePose(); }

    // set the target pose as rbd pose directly
    void setTargetPose(const ct::rbd::RigidBodyPose& rbdPose)
    {
        goalCostTerm_->setReferencePosition(rbdPose.position().toImplementation());
        goalCostTerm_->setReferenceOrientation(rbdPose.getRotationQuaternion().toImplementation());
    }

    // set the target pose as separate position and quaternion
    void setTargetPose(const core::StateVector<3>& w_pos_des, const Eigen::Quaterniond& w_q_des)
    {
        goalCostTerm_->setReferencePosition(w_pos_des);
        goalCostTerm_->setReferenceOrientation(w_q_des);
    }

    // set the gart pose as euler angles directly
    void setTargetPose(const core::StateVector<3>& w_pos_des, const Eigen::Matrix3d& eulerXyz)
    {
        goalCostTerm_->setReferencePosition(w_pos_des);
        goalCostTerm_->setReferenceOrientation(
            Eigen::Quaterniond(Eigen::AngleAxisd(eulerXyz(0), Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(eulerXyz(1), Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(eulerXyz(2), Eigen::Vector3d::UnitZ())));
    }

    //! evaluate cost function
    virtual SCALAR eval() override
    {
        if (w_ != nullptr)
        {
            SCALAR val =
                goalCostTerm_->evaluate(w_->getOptimizationVars(), Eigen::Matrix<double, NJOINTS, 1>::Zero(), 0.0);

            if (jointRefTerm_)
                val +=
                    jointRefTerm_->evaluate(w_->getOptimizationVars(), Eigen::Matrix<double, NJOINTS, 1>::Zero(), 0.0);

            return val;
        }
        else
            throw std::runtime_error("IKCostEvaluator: optimization vector not set.");
    }

    //! retrieve first-order derivative
    virtual void evalGradient(size_t grad_length, Eigen::Map<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>>& grad) override
    {
        assert(grad_length == NJOINTS);

        if (w_ != nullptr)
        {
            grad = goalCostTerm_->stateDerivative(
                w_->getOptimizationVars(), Eigen::Matrix<double, NJOINTS, 1>::Zero(), 0.0);

            if (jointRefTerm_)
                grad += jointRefTerm_->stateDerivative(
                    w_->getOptimizationVars(), Eigen::Matrix<double, NJOINTS, 1>::Zero(), 0.0);
        }
        else
            throw std::runtime_error("IKCostEvaluator: optimization vector not set.");
    }


    // todo make lower triangular (when convenient)
    //! create sparsity pattern for the hessian (simply fill up completely)
    virtual void getSparsityPatternHessian(Eigen::VectorXi& iRow, Eigen::VectorXi& jCol) override
    {
        iRow.resize(NJOINTS * NJOINTS);
        jCol.resize(NJOINTS * NJOINTS);

        size_t count = 0;
        for (size_t i = 0; i < NJOINTS; i++)
        {
            for (size_t j = 0; j < NJOINTS; j++)
            {
                iRow(count) = i;
                jCol(count) = j;
                count++;
            }
        }
    }


    // todo make lower triangular (when convenient)
    //! retrieve second order derivative
    void sparseHessianValues(const Eigen::VectorXd& jointAngles,
        const Eigen::VectorXd& obj_fac,
        Eigen::VectorXd& hes) override
    {
        hes.resize(NJOINTS * NJOINTS);

        if (w_ != nullptr)
        {
            // map hessian value-vector to matrix
            Eigen::Map<Eigen::Matrix<SCALAR, NJOINTS, NJOINTS>> Hmat(hes.data(), NJOINTS, NJOINTS);

            Hmat = goalCostTerm_->stateSecondDerivative(
                w_->getOptimizationVars(), Eigen::Matrix<double, NJOINTS, 1>::Zero(), 0.0);

            if (jointRefTerm_)
                Hmat += jointRefTerm_->stateSecondDerivative(
                    w_->getOptimizationVars(), Eigen::Matrix<double, NJOINTS, 1>::Zero(), 0.0);

            if (ikRegularizer_)
                Hmat += ikRegularizer_->computeRegularizer(w_->getOptimizationVars());

            Hmat *= obj_fac(0, 0);
        }
        else
            throw std::runtime_error("IKCostEvaluator: optimization vector not set.");
    }


private:
    std::shared_ptr<ct::optcon::tpl::OptVector<SCALAR>> w_;

    std::shared_ptr<ct::rbd::TermTaskspaceGeometricJacobian<KINEMATICS, NJOINTS, NJOINTS>> goalCostTerm_;

    std::shared_ptr<ct::optcon::TermQuadratic<NJOINTS, NJOINTS, SCALAR>> jointRefTerm_;

    std::shared_ptr<IKRegularizerBase>
        ikRegularizer_;  // optional regularizer for Hessian matrix! (potentially goes away)
};

}  // namespace rbd
}  // namespace ct
