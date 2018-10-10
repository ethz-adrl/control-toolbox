/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/optcon/nlp/Nlp>

#include "IKCostEvaluator.h"
#include "IKConstraintContainer.h"
#include "../InverseKinematicsBase.h"

namespace ct {
namespace rbd {

/*!
 * todo: set more meaningful initial guess
 * todo implement cost evaluator
 * todo implement constraints
 */
template <typename KINEMATICS, typename SCALAR = double>
class IKNLP : public ct::optcon::tpl::Nlp<SCALAR>, public ct::rbd::InverseKinematicsBase<KINEMATICS::NJOINTS, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using InverseKinematicsBase = ct::rbd::InverseKinematicsBase<KINEMATICS::NJOINTS, SCALAR>;
    using JointPosition_t = typename InverseKinematicsBase::JointPosition_t;
    using JointPositionsVector_t = typename InverseKinematicsBase::JointPositionsVector_t;
    using RigidBodyPoseTpl = typename InverseKinematicsBase::RigidBodyPoseTpl;

    //! constructor
    IKNLP(std::shared_ptr<ct::rbd::IKCostEvaluator<KINEMATICS, SCALAR>> costEvaluator,
        const JointPosition_t& lowerBound,
        const JointPosition_t& upperBound)
    {
        // the number of optimization variables is the number of robot joints
        this->optVariables_ = std::shared_ptr<ct::optcon::tpl::OptVector<SCALAR>>(
            new ct::optcon::tpl::OptVector<SCALAR>(KINEMATICS::NJOINTS));
        // trivial initial guess
        this->optVariables_->setZero();

        // set the cost evaluator
        this->costEvaluator_ = costEvaluator;
        std::static_pointer_cast<ct::rbd::IKCostEvaluator<KINEMATICS, SCALAR>>(this->costEvaluator_)
            ->setOptVector(this->optVariables_);

        this->constraints_ = std::shared_ptr<IKConstraintsContainer<KINEMATICS, SCALAR>>(
            new IKConstraintsContainer<KINEMATICS, SCALAR>(this->optVariables_, lowerBound, upperBound));
    }

    //! default destructor
    virtual ~IKNLP() override = default;

    virtual void updateProblem() override { /* do nothing */}

    virtual bool computeInverseKinematics(JointPositionsVector_t& ikSolutions,
        const RigidBodyPoseTpl& eeBasePose,
        const std::vector<size_t>& freeJoints = std::vector<size_t>()) const override
    {
        std::static_pointer_cast<ct::rbd::IKCostEvaluator<KINEMATICS, SCALAR>>(this->costEvaluator_)
            ->setTargetPose(eeBasePose);

        throw std::runtime_error("iknlp not implemented yet.");
        // todo implement
    }

    virtual bool computeInverseKinematics(JointPositionsVector_t& ikSolutions,
        const RigidBodyPoseTpl& eeWorldPose,
        const RigidBodyPoseTpl& baseWorldPose,
        const std::vector<size_t>& freeJoints = std::vector<size_t>()) const override
    {
        throw std::runtime_error("iknlp not implemented yet.");
        // todo implement
    }

    //! set an initial guess for the numerical routine
    void setInitialGuess(const typename ct::rbd::JointState<KINEMATICS::NJOINTS>::Position& q_init)
    {
        throw std::runtime_error("IKNLP -- no initial guess setting implemented yet.");
        //        this->optVariables_->setInitGuess(x_init_guess, u_init_guess);
    }

    //! print the solution to command line
    void printSolution()
    {
        std::cout << "Inverse kinematics solution: " << std::endl
                  << this->optVariables_->getOptimizationVars().transpose() << std::endl;
    }

private:
};

}  // rbd
}  // ct
