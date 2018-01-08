/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/optcon/optcon.h>
#include <ct/rbd/rbd.h>
#include "../exampleDir.h"

#include <ct/models/InvertedPendulum/InvertedPendulum.h>

using namespace ct::rbd;

const size_t njoints            = ct::rbd::InvertedPendulum::Kinematics::NJOINTS;
const size_t actuator_state_dim = 1;

typedef ct::rbd::InvertedPendulum::tpl::Dynamics<double> IPDynamics;
typedef ct::rbd::FixBaseFDSystem<IPDynamics, actuator_state_dim, false> IPSystem;

typedef ct::core::LinearSystem<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM, double> LinearSystem;

using InvertedPendulumNLOC = FixBaseNLOC<IPSystem, actuator_state_dim, double>;

class MPCSimulator : public ct::core::ControlSimulator<IPSystem>
{
public:
    MPCSimulator(ct::core::Time sim_dt,
        ct::core::Time control_dt,
        const ct::core::StateVector<STATE_DIM>& x0,
        std::shared_ptr<IPSystem> ip_system,
        ct::optcon::MPC<ct::optcon::NLOptConSolver<STATE_DIM, CONTROL_DIM>>& mpc)
        : ct::core::ControlSimulator<IPSystem>(sim_dt, control_dt, x0, ip_system), mpc_(mpc)
    {
        controller_.reset(new ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM>);
    }

    void finishSystemIteration(ct::core::Time sim_time) override
    {
        control_mtx_.lock();
        system_->setController(controller_);
        control_mtx_.unlock();
    }

    void prepareControllerIteration(ct::core::Time sim_time) override
    {
        mpc_.prepareIteration(sim_time);
    }

    void finishControllerIteration(ct::core::Time sim_time) override
    {
        state_mtx_.lock();
        ct::core::StateVector<STATE_DIM> x_temp = x_;
        state_mtx_.unlock();

        std::shared_ptr<ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM>> new_controller(
            new ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM>);
        bool success = mpc_.finishIteration(x_temp, sim_time, *new_controller, controller_ts_);
        if (!success) throw "Failed to finish iteration.";

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
        std::string workingDirectory = ct::models::exampleDir + "/InvertedPendulum";

        std::string configFile       = workingDirectory + "/solver.info";
        std::string costFunctionFile = workingDirectory + "/cost.info";

        std::shared_ptr<ct::rbd::SEADynamicsFirstOrder<njoints, double>> actuatorDynamics(
            new ct::rbd::SEADynamicsFirstOrder<njoints, double>(160.0));
        std::shared_ptr<IPSystem> ipSystem(new IPSystem(actuatorDynamics));

        // NLOC settings
        ct::optcon::NLOptConSettings nloc_settings;
        nloc_settings.load(configFile, verbose, "ilqr");

        std::shared_ptr<ct::optcon::TermQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM, double, double>>
            termQuadInterm(new ct::optcon::TermQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM, double, double>);
        termQuadInterm->loadConfigFile(costFunctionFile, "term0", verbose);

        std::shared_ptr<ct::optcon::TermQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM, double, double>>
            termQuadFinal(new ct::optcon::TermQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM, double, double>);
        termQuadFinal->loadConfigFile(costFunctionFile, "term1", verbose);

        std::shared_ptr<ct::optcon::CostFunctionAnalytical<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>> newCost(
            new ct::optcon::CostFunctionAnalytical<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>);
        size_t intTermID   = newCost->addIntermediateTerm(termQuadInterm);
        size_t finalTermID = newCost->addFinalTerm(termQuadFinal);

        ct::core::Time timeHorizon;
        InvertedPendulumNLOC::FeedbackArray::value_type fbD;
        ct::rbd::tpl::JointState<njoints, double> x0;
        ct::rbd::tpl::JointState<njoints, double> xf;

        ct::core::loadScalar(configFile, "timeHorizon", timeHorizon);
        ct::core::loadMatrix(costFunctionFile, "K_init", fbD);
        ct::core::loadMatrix(costFunctionFile, "x_0", x0.toImplementation());
        ct::core::loadMatrix(costFunctionFile, "term1.weights.x_des", xf.toImplementation());
        ct::core::StateVector<IPSystem::STATE_DIM> x0full = IPSystem::toFullState(x0.toImplementation());

        std::shared_ptr<LinearSystem> linSystem = nullptr;
        ct::optcon::OptConProblem<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM> optConProblem(
            timeHorizon, x0full, ipSystem, newCost, linSystem);
        InvertedPendulumNLOC nloc_solver(newCost, nloc_settings, ipSystem, verbose, linSystem);

        int K = nloc_solver.getSettings().computeK(timeHorizon);

        InvertedPendulumNLOC::StateVectorArray stateRefTraj(K + 1, x0full);
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

        // nloc_solver.retrieveLastRollout();

        ct::optcon::NLOptConSettings ilqr_settings_mpc(nloc_solver.getSettings());
        ilqr_settings_mpc.max_iterations = 1;
        ilqr_settings_mpc.printSummary   = false;

        ct::optcon::mpc_settings mpc_settings;
        mpc_settings.stateForwardIntegration_    = false;
        mpc_settings.postTruncation_             = false;
        mpc_settings.measureDelay_               = false;
        mpc_settings.delayMeasurementMultiplier_ = 1.0;
        mpc_settings.mpc_mode                    = ct::optcon::MPC_MODE::CONSTANT_RECEDING_HORIZON;
        mpc_settings.coldStart_                  = false;
        mpc_settings.minimumTimeHorizonMpc_      = 3.0;

        ct::optcon::MPC<ct::optcon::NLOptConSolver<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>> ilqr_mpc(
            optConProblem, ilqr_settings_mpc, mpc_settings);
        ilqr_mpc.setInitialGuess(initialSolution);

        MPCSimulator mpc_sim(1e-3, 1e-2, x0full, ipSystem, ilqr_mpc);
        std::cout << "simulating 3 seconds" << std::endl;
        mpc_sim.simulate(3);
        mpc_sim.finish();

        ilqr_mpc.printMpcSummary();

    } catch (std::runtime_error& e)
    {
        std::cout << "Exception caught: " << e.what() << std::endl;
    }
}
