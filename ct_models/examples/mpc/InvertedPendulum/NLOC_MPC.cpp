/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/rbd/rbd.h>
#include "../../exampleDir.h"

#include <ct/models/InvertedPendulum/InvertedPendulum.h>

using namespace ct::rbd;

const size_t njoints = ct::rbd::InvertedPendulum::Kinematics::NJOINTS;
const size_t actuator_state_dim = 1;

using RobotState_t = ct::rbd::FixBaseRobotState<njoints, actuator_state_dim>;
static const size_t state_dim = RobotState_t::NSTATE;
static const size_t control_dim = njoints;

using IPDynamics = ct::rbd::InvertedPendulum::tpl::Dynamics<double>;
using IPSystem = ct::rbd::FixBaseFDSystem<IPDynamics, actuator_state_dim, false>;
using LinearSystem = ct::core::LinearSystem<state_dim, control_dim, double>;

using InvertedPendulumNLOC = FixBaseNLOC<IPSystem>;

class MPCSimulator : public ct::core::ControlSimulator<IPSystem>
{
public:
    MPCSimulator(ct::core::Time sim_dt,
        ct::core::Time control_dt,
        const RobotState_t& x0,
        std::shared_ptr<IPSystem> ip_system,
        ct::optcon::MPC<ct::optcon::NLOptConSolver<STATE_DIM, CONTROL_DIM>>& mpc)
        : ct::core::ControlSimulator<IPSystem>(sim_dt, control_dt, x0.toStateVector(), ip_system), mpc_(mpc)
    {
        controller_.reset(new ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM>);
    }

    void finishSystemIteration(ct::core::Time sim_time) override
    {
        control_mtx_.lock();
        system_->setController(controller_);
        control_mtx_.unlock();
    }

    void prepareControllerIteration(ct::core::Time sim_time) override { mpc_.prepareIteration(sim_time); }
    void finishControllerIteration(ct::core::Time sim_time) override
    {
        state_mtx_.lock();
        ct::core::StateVector<STATE_DIM> x_temp = x_;
        state_mtx_.unlock();

        std::shared_ptr<ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM>> new_controller(
            new ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM>);

        bool success = mpc_.finishIteration(x_temp, sim_time, *new_controller, controller_ts_);

        if (!success)
            throw std::runtime_error("Failed to finish MPC iteration.");

        control_mtx_.lock();
        controller_ = new_controller;
        control_mtx_.unlock();
    }

private:
    ct::optcon::MPC<ct::optcon::NLOptConSolver<STATE_DIM, CONTROL_DIM>>& mpc_;
    ct::core::Time controller_ts_;
};

int main(int argc, char* argv[])
{
    const bool verbose = true;
    try
    {
        std::string workingDirectory = ct::models::exampleDir + "/mpc/InvertedPendulum";

        std::string configFile = workingDirectory + "/solver.info";
        std::string costFunctionFile = workingDirectory + "/cost.info";

        const double k_spring = 160;
        const double gear_ratio = 50;

        std::shared_ptr<ct::rbd::SEADynamicsFirstOrder<njoints>> actuatorDynamics(
            new ct::rbd::SEADynamicsFirstOrder<njoints>(k_spring, gear_ratio));
        std::shared_ptr<IPSystem> ipSystem(new IPSystem(actuatorDynamics));

        // NLOC settings
        ct::optcon::NLOptConSettings nloc_settings;
        nloc_settings.load(configFile, verbose, "ilqr");

        std::shared_ptr<ct::optcon::TermQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>> termQuadInterm(
            new ct::optcon::TermQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>);
        termQuadInterm->loadConfigFile(costFunctionFile, "term0", verbose);

        std::shared_ptr<ct::optcon::TermQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>> termQuadFinal(
            new ct::optcon::TermQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>);
        termQuadFinal->loadConfigFile(costFunctionFile, "term1", verbose);

        std::shared_ptr<ct::optcon::CostFunctionAnalytical<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>> newCost(
            new ct::optcon::CostFunctionAnalytical<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>);
        size_t intTermID = newCost->addIntermediateTerm(termQuadInterm);
        /*size_t finalTermID = */ newCost->addFinalTerm(termQuadFinal);

        ct::core::Time timeHorizon;
        InvertedPendulumNLOC::FeedbackArray::value_type fbD;
        RobotState_t x0;
        RobotState_t xf;

        ct::core::loadScalar(configFile, "timeHorizon", timeHorizon);
        ct::core::loadMatrix(costFunctionFile, "K_init", fbD);
        RobotState_t::state_vector_t xftemp, x0temp;
        ct::core::loadMatrix(costFunctionFile, "x_0", x0temp);
        ct::core::loadMatrix(costFunctionFile, "term1.weights.x_des", xftemp);
        x0.fromStateVector(x0temp);
        xf.fromStateVector(xftemp);

        std::shared_ptr<LinearSystem> linSystem = nullptr;

        ct::optcon::ContinuousOptConProblem<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM> optConProblem(
            timeHorizon, x0.toStateVector(), ipSystem, newCost, linSystem);

        InvertedPendulumNLOC nloc_solver(newCost, nloc_settings, ipSystem, verbose, linSystem);

        int K = nloc_solver.getSettings().computeK(timeHorizon);

        InvertedPendulumNLOC::StateVectorArray stateRefTraj(K + 1, x0.toStateVector());
        InvertedPendulumNLOC::FeedbackArray fbTrajectory(K, -fbD);
        InvertedPendulumNLOC::ControlVectorArray ffTrajectory(K, InvertedPendulumNLOC::ControlVector::Zero());

        int initType = 0;
        ct::core::loadScalar(configFile, "initType", initType);

        switch (initType)
        {
            case 0:  // steady state
            {
                ct::core::ControlVector<IPSystem::CONTROL_DIM> uff_ref;
                nloc_solver.initializeSteadyPose(x0, timeHorizon, K, uff_ref, -fbD);

                std::vector<
                    std::shared_ptr<ct::optcon::CostFunctionQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>>>&
                    inst1 = nloc_solver.getSolver()->getCostFunctionInstances();

                for (size_t i = 0; i < inst1.size(); i++)
                {
                    inst1[i]->getIntermediateTermById(intTermID)->updateReferenceControl(uff_ref);
                }
                break;
            }
            case 1:  // linear interpolation
            {
                nloc_solver.initializeDirectInterpolation(x0, xf, timeHorizon, K, -fbD);
                break;
            }
            default:
            {
                throw std::runtime_error("illegal init type");
                break;
            }
        }

        std::cout << "waiting 1 second for begin" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        nloc_solver.solve();
        ct::core::StateFeedbackController<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM> initialSolution =
            nloc_solver.getSolution();
        InvertedPendulumNLOC::StateVectorArray x_nloc = initialSolution.x_ref();


        ct::optcon::NLOptConSettings ilqr_settings_mpc(nloc_solver.getSettings());
        ilqr_settings_mpc.max_iterations = 1;
        ilqr_settings_mpc.printSummary = false;

        ct::optcon::mpc_settings mpc_settings;
        mpc_settings.stateForwardIntegration_ = false;
        mpc_settings.postTruncation_ = false;
        mpc_settings.measureDelay_ = false;
        mpc_settings.delayMeasurementMultiplier_ = 1.0;
        mpc_settings.mpc_mode = ct::optcon::MPC_MODE::CONSTANT_RECEDING_HORIZON;
        mpc_settings.coldStart_ = false;
        mpc_settings.minimumTimeHorizonMpc_ = 3.0;

        ct::optcon::MPC<ct::optcon::NLOptConSolver<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>> ilqr_mpc(
            optConProblem, ilqr_settings_mpc, mpc_settings);
        ilqr_mpc.setInitialGuess(initialSolution);

        ct::core::Time sim_dt = 1e-3;
        ct::core::Time control_dt = 1e-2;

        MPCSimulator mpc_sim(sim_dt, control_dt, x0, ipSystem, ilqr_mpc);
        std::cout << "simulating 3 seconds" << std::endl;
        mpc_sim.simulate(3.0);
        mpc_sim.finish();

        ilqr_mpc.printMpcSummary();

    } catch (std::runtime_error& e)
    {
        std::cout << "Exception caught: " << e.what() << std::endl;
    }
}
