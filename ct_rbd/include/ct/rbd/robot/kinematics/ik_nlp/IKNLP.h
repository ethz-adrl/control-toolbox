/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/optcon/nlp/Nlp>

#include "IKCostEvaluator.h"

namespace ct {
namespace rbd {

/*!
 * todo: set more meaningful initial guess
 * todo implement cost evaluator
 * todo implement constraints
 */
template <typename KINEMATICS, typename SCALAR = double>
class IKNLP : public ct::optcon::tpl::Nlp<SCALAR>
{
public:

	//! constructor
    IKNLP(std::shared_ptr<ct::rbd::IKCostEvaluator<KINEMATICS, SCALAR>> costEvaluator)
    {
    	// the number of optimization variables is the number of robot joints
        this->optVariables_ = std::shared_ptr<ct::optcon::tpl::OptVector<SCALAR>>(
            new ct::optcon::tpl::OptVector<SCALAR>(KINEMATICS::NJOINTS));
        // trivial initial guess
        this->optVariables_->setZero();

        // set the cost evaluator
        this->costEvaluator_ = costEvaluator;
        this->costEvaluator->setOptVector(this->optVariables_);

        this->constraints_ = nullptr; // todo replace
//        this->constraints_ = std::shared_ptr<ct::rbd::IKConstraintsContainer<SCALAR>> (
//            new ct::rbd::IKConstraintsContainer<SCALAR>(this->optVariables_));
//
    }

    //! default destructor
    virtual ~IKNLP() override = default;

    virtual void updateProblem() override { /* do nothing */ }

    //! set an initial guess for the numerical routine
    void setInitialGuess(const typename ct::rbd::JointState<KINEMATICS::NJOINTS>::Position& q_init)
    {
    	throw std::runtime_error("IKNLP -- no initial guess setting implemented yet.");
//        this->optVariables_->setInitGuess(x_init_guess, u_init_guess);
    }

    //! print the solution to command line
    void printSolution()
    {
        std::cout << "Inverse kinematics solution: " << std::endl << this->optVariables_->getOptimizationVars().transpose() << std::endl;
    }

private:
};

}  // rbd
}  // ct
