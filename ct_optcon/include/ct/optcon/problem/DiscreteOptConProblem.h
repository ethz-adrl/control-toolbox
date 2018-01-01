/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DiscreteOptConProblem : public ct::optcon::OptConProblemBase<STATE_DIM,
                                  CONTROL_DIM,
                                  core::DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>,
                                  core::DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>,
                                  SCALAR>
{
    typedef ct::optcon::OptConProblemBase<STATE_DIM,
        CONTROL_DIM,
        core::DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>,
        core::DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>,
        SCALAR>
        Base;

    typedef typename Base::time_t time_t;

    //TODO add necessary constructors

    /*!
     * @brief Construct a discrete Optimal Control Problem from a continuous one
     *
     * @param continuousProblem the continuous Optimal Control Problem
     * @param dt the discretization time interval
     */
    DiscreteOptConProblem(const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& continuousProblem, const double dt)
    {
        continuousProblem.verify();  // ensure the continuous problem is complete

        // TODO: we might not need this at all
        std::shared_ptr<ct::core::SystemDiscretizer<STATE_DIM, CONTROL_DIM>> discreteSys;
        discreteSys.reset(
            new ct::core::SystemDiscretizer<STATE_DIM, CONTROL_DIM>(continuousProblem.getNonlinearSystem(), dt));

        setNonlinearSystem(discreteSys);

        //TODO need controller
        std::shared_ptr<ct::core::SensitivityIntegrator<STATE_DIM, CONTROL_DIM>> discreteLinSys;
        discreteLinSys.reset(
            new ct::core::SensitivityIntegrator<STATE_DIM, CONTROL_DIM>(dt, continuousProblem.getLinearSystem()));
        setLinearSystem(discreteLinSys);

        //TODO discretize the cost function
        //setCostFunction(...);
        throw std::runtime_error("Not yet implemented");

        setBoxConstraints(continuousProblem.getBoxConstraints());
        setGeneralConstraints(continuousProblem.getGeneralConstraints());
        setInitialState(continuousProblem.getInitialState());
        setTimeHorizon(static_cast<time_t>(continuousProblem.getTimeHorizon() / dt));
    }
};

}  // namespace optcon
}  // namespace ct
