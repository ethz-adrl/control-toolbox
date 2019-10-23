/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <chrono>

#include <gtest/gtest.h>


/*!
 * This test implements iLQR and GNMS for a symplectic system.
 *
 * \example SymplecticTest.h
 *
 * \note visit the tutorial for a more intuitive example.
 *
 * \warning The HPIPM solver is not included in this unit test.
 */


namespace ct {
namespace optcon {
namespace example {

using namespace ct::core;
using namespace ct::optcon;

using std::shared_ptr;

const size_t state_dim = 2;    // position, velocity
const size_t control_dim = 1;  // force

const double kStiffness = 1;

//! Dynamics class for the GNMS unit test
class Dynamics : public SymplecticSystem<1, 1, control_dim>
{
public:
    Dynamics() : SymplecticSystem<1, 1, control_dim>(SYSTEM_TYPE::SECOND_ORDER) {}
    void computePdot(const StateVector<2>& x,
        const StateVector<1>& v,
        const ControlVector<1>& control,
        StateVector<1>& pDot) override
    {
        pDot = v;
    }

    virtual void computeVdot(const StateVector<2>& x,
        const StateVector<1>& p,
        const ControlVector<1>& control,
        StateVector<1>& vDot) override
    {
        vDot(0) = control(0) - kStiffness * p(0) * p(0);  // mass is 1 k
    }

    Dynamics* clone() const override { return new Dynamics(); };
};


//! Linear system class for the GNMS unit test
class LinearizedSystem : public LinearSystem<state_dim, control_dim>
{
public:
    state_matrix_t A_;
    state_control_matrix_t B_;


    const state_matrix_t& getDerivativeState(const StateVector<state_dim>& x,
        const ControlVector<control_dim>& u,
        const double t = 0.0) override
    {
        A_ << 0, 1, -2 * x(0) * kStiffness, 0;
        return A_;
    }

    const state_control_matrix_t& getDerivativeControl(const StateVector<state_dim>& x,
        const ControlVector<control_dim>& u,
        const double t = 0.0) override
    {
        B_ << 0, 1;
        return B_;
    }

    LinearizedSystem* clone() const override { return new LinearizedSystem(); };
};

//! Create a cost function for the GNMS unit test
std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> createCostFunction(Eigen::Vector2d& x_final)
{
    Eigen::Matrix2d Q;
    Q << 0, 0, 0, 1;

    Eigen::Matrix<double, 1, 1> R;
    R << 100.0;

    Eigen::Vector2d x_nominal = Eigen::Vector2d::Zero();
    Eigen::Matrix<double, 1, 1> u_nominal = Eigen::Matrix<double, 1, 1>::Zero();

    Eigen::Matrix2d Q_final;
    Q_final << 10, 0, 0, 10;

    std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> quadraticCostFunction(
        new CostFunctionQuadraticSimple<state_dim, control_dim>(Q, R, x_nominal, u_nominal, x_final, Q_final));

    return quadraticCostFunction;
}


void symplecticTest()
{
    std::cout << "setting up problem " << std::endl;

    typedef NLOptConSolver<state_dim, control_dim, state_dim / 2, state_dim / 2> NLOptConSolver;

    Eigen::Vector2d x_final;
    x_final << 2, 0;

    NLOptConSettings gnms_settings;
    gnms_settings.nThreads = 1;
    gnms_settings.epsilon = 0.0;
    gnms_settings.max_iterations = 1;
    gnms_settings.recordSmallestEigenvalue = false;
    gnms_settings.min_cost_improvement = 1e-6;
    gnms_settings.fixedHessianCorrection = false;
    gnms_settings.dt = 0.05;
    gnms_settings.K_sim = 50;
    gnms_settings.K_shot = 1;
    gnms_settings.integrator = ct::core::IntegrationType::EULER_SYM;
    //	gnms_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
    gnms_settings.useSensitivityIntegrator = true;
    gnms_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::GNMS;
    gnms_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;
    gnms_settings.lineSearchSettings.type = LineSearchSettings::TYPE::NONE;
    gnms_settings.loggingPrefix = "GNMS";
    gnms_settings.printSummary = false;


    NLOptConSettings ilqr_settings = gnms_settings;
    ilqr_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;
    ilqr_settings.loggingPrefix = "ILQR";


    shared_ptr<ControlledSystem<state_dim, control_dim>> nonlinearSystem(new Dynamics);
    shared_ptr<LinearSystem<state_dim, control_dim>> analyticLinearSystem(new LinearizedSystem);
    shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction = createCostFunction(x_final);

    // times
    ct::core::Time tf = 3.0;
    size_t nSteps = std::round(tf / gnms_settings.dt);

    // provide initial guess
    StateVector<state_dim> initState;
    initState.setZero();  //initState(1) = 1.0;
    StateVectorArray<state_dim> x0(nSteps + 1, initState);
    ControlVector<control_dim> uff;
    uff << kStiffness * initState(0) * initState(0);
    ControlVectorArray<control_dim> u0(nSteps, uff);
    for (size_t i = 0; i < nSteps + 1; i++)
    {
        //			x0 [i] = x_final*double(i)/double(nSteps);
    }

    FeedbackArray<state_dim, control_dim> u0_fb(nSteps, FeedbackMatrix<state_dim, control_dim>::Zero());
    ControlVectorArray<control_dim> u0_ff(nSteps, ControlVector<control_dim>::Zero());

    NLOptConSolver::Policy_t initController(x0, u0, u0_fb, gnms_settings.dt);

    // construct single-core single subsystem OptCon Problem
    ContinuousOptConProblem<state_dim, control_dim> optConProblem(
        tf, x0[0], nonlinearSystem, costFunction, analyticLinearSystem);


    //	std::cout << "initializing gnms solver" << std::endl;
    NLOptConSolver gnms(optConProblem, gnms_settings);
    NLOptConSolver ilqr(optConProblem, ilqr_settings);


    gnms.configure(gnms_settings);
    gnms.setInitialGuess(initController);

    ilqr.configure(ilqr_settings);
    ilqr.setInitialGuess(initController);

    //	std::cout << "running gnms solver" << std::endl;

    size_t numIterations = 0;

    while (numIterations < 50)
    {
        gnms.runIteration();
        ilqr.runIteration();

        // test trajectories
        StateTrajectory<state_dim> xRollout = gnms.getStateTrajectory();
        ControlTrajectory<control_dim> uRollout = gnms.getControlTrajectory();

        //		// test trajectories
        //		StateTrajectory<state_dim> xRollout = ilqr.getStateTrajectory();
        //		ControlTrajectory<control_dim> uRollout = ilqr.getControlTrajectory();

        numIterations++;

        //		std::cout<<"x final iLQR: " << xRollout.back().transpose() << std::endl;
        //		std::cout<<"u final iLQR: " << uRollout.back().transpose() << std::endl;
    }
}


}  // namespace example
}  // namespace optcon
}  // namespace ct
