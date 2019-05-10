/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#ifndef INCLUDE_CT_RBD_SLQ_FLOATINGBASESLQ_H_
#define INCLUDE_CT_RBD_SLQ_FLOATINGBASESLQ_H_

#include <ct/core/systems/linear/SystemLinearizer.h>

#include <ct/optcon/ilqg/iLQGMP.hpp>

#include <ct/rbd/systems/FloatingBaseFDSystem.h>


namespace ct {
namespace rbd {

/**
 * \brief SLQ for floating base systems without an explicit contact model.
 * The contact constraint is enforced via a cost function
 */
template <class RBDDynamics>
class FloatingBaseSLQ
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const bool quatIntegration = false;
    static const bool eeForcesAreControlInputs = true;

    typedef FloatingBaseFDSystem<RBDDynamics, false, true> FBSystem;
    typedef ct::core::LinearSystem<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> LinearizedSystem;
    typedef ct::core::SystemLinearizer<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> SystemLinearizer;

    typedef ct::optcon::iLQGMP<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> iLQG;
    typedef ct::optcon::iLQGMP<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> iLQGMP;

    typedef ct::optcon::CostFunctionAnalytical<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> CostFunction;
    typedef ct::rbd::TermTaskspace<FBSystem::Kinematics, true, FBSystem::STATE_DIM, FBSystem::CONTROL_DIM>
        TaskSpaceTerm;

    typedef iLQG::StateTrajectory StateTrajectory;


    FloatingBaseSLQ(std::shared_ptr<FBSystem> system,
        const std::string& costFunctionFile,
        std::shared_ptr<LinearizedSystem> linearizedSystem = nullptr,
        bool useMP = true)
        : system_(system), linearizedSystem_(linearizedSystem), useMP_(useMP), iteration_(0)
    {
        // if no linearized system provided, use general system linearizer
        if (linearizedSystem_ == nullptr)
            linearizedSystem_ = std::shared_ptr<LinearizedSystem>(new SystemLinearizer(system));

        setupCostFunction(costFunctionFile);

        ilqg_ = std::shared_ptr<iLQG>(new iLQG(system_, linearizedSystem_, costFunction_));
        ilqgMp_ = std::shared_ptr<iLQGMP>(new iLQGMP(system_, linearizedSystem_, costFunction_));
    }

    bool runIteration(StateTrajectory& stateTrajectory)
    {
        bool foundBetter;

        if (useMP_)
        {
            foundBetter = ilqgMp_->runIteration(ilqg_settings_, line_search_settings_);
            stateTrajectory = ilqgMp_->retrieveLastRollout();
        }
        else
        {
            foundBetter = ilqg_->runIteration(ilqg_settings_, line_search_settings_);
            stateTrajectory = ilqg_->retrieveLastRollout();
        }

        iteration_++;
        return foundBetter;
    }

    ilqg_settings_t iLQGSettings() { return ilqg_settings_; }
private:
    void setupCostFunction(const std::string& filename)
    {
        costFunction_ = std::shared_ptr<CostFunction>(new CostFunction(filename, true));

        for (size_t i = 0; i < N_EE; i++)
        {
            eePosTerms_.push_back(new TaskSpaceTerm(filename, "eePos" + std::to_string(i), true));
            costFunction_->addIntermediateTerm(eePosTerms_.back());
        }
    }

    std::shared_ptr<FBSystem> system_;
    std::shared_ptr<LinearizedSystem> linearizedSystem_;

    std::shared_ptr<iLQG> ilqg_;
    std::shared_ptr<iLQG> ilqgMp_;

    std::shared_ptr<CostFunction> costFunction_;
    std::vector<std::shared_ptr<TaskSpaceTerm>> eePosTerms_;

    ilqg_settings_t ilqg_settings_;
    line_search_settings_t line_search_settings_;

    bool useMP_;

    size_t iteration_;
};
}
}

#endif /* INCLUDE_CT_RBD_SLQ_FLOATINGBASESLQ_H_ */
