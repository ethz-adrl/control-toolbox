#include <gtest/gtest.h>

#include <ct/core/core.h>
#include <ct/optcon/optcon.h>

#include "discreteTestSystem.hpp"

namespace ct {
namespace optcon {

/*!
 * This unit test applies Direct Multiple Shooting to a discrete nonlinear test system.
 * \example discreteProblemDmsTest.cpp
 */

TEST(DiscreteDmsTest, IPOPT)
{
    using sysD_t = ct::optcon::tpl::TestDiscreteNonlinearSystem<double>;
    const size_t state_dim = sysD_t::STATE_DIM;
    const size_t control_dim = sysD_t::CONTROL_DIM;
    using ADCGScalar = ct::core::DiscreteSystemLinearizerADCG<state_dim, control_dim>::ADCGScalar;
    using sysADCG_t = ct::optcon::tpl::TestDiscreteNonlinearSystem<ADCGScalar>;

    using AD_ValueType = ADCGScalar::value_type;

    using ADCGlinearizer_t = ct::core::DiscreteSystemLinearizerADCG<state_dim, control_dim>;

    constexpr double rate = 2.0;
    std::shared_ptr<sysD_t> sys(new sysD_t(rate));
    std::shared_ptr<sysADCG_t> sysADCG(new sysADCG_t(AD_ValueType(rate)));
    std::shared_ptr<ADCGlinearizer_t> lin(new ADCGlinearizer_t(sysADCG));

    std::cout << "compiling linear system JIT..." << std::endl;
    lin->compileJIT("ADCGCodegenLib_testDiscreteNonlinSys");
    std::cout << "... done!" << std::endl;

    ct::optcon::DmsSettings settings;

    settings.solverSettings_.solverType_ = ct::optcon::NlpSolverSettings::IPOPT;
    settings.solverSettings_.ipoptSettings_.derivativeTest_ = "first-order";
    settings.solverSettings_.ipoptSettings_.checkDerivativesForNaninf_ = "yes";
    settings.solverSettings_.ipoptSettings_.derivativeTestPrintAll_ = "yes";
    settings.solverSettings_.ipoptSettings_.derivativeTestTol_ = 1e-5;
    settings.N_ = 50; // number of shots
    settings.T_ = 1.0; // time horizon
    settings.nThreads_ = 1;
    settings.dt_sim_ = 0.01;


    sysD_t::state_vector_t x_0, x_final;
    x_0.setRandom();
    x_final.setZero();
    sysD_t::control_vector_t u_des;
    u_des.setZero();
    ct::core::StateMatrix<state_dim, double> Q, Q_final;
    Q.setIdentity();
    Q_final.setIdentity();
    ct::core::ControlMatrix<control_dim, double> R;
    R.setIdentity();

    using cost_t = ct::optcon::CostFunctionQuadratic<state_dim, control_dim>;
    std::shared_ptr<cost_t> cost(
        new ct::optcon::CostFunctionQuadraticSimple<state_dim, control_dim>(Q, R, x_final, u_des, x_final, Q_final));

    ct::optcon::DiscreteOptConProblem<state_dim, control_dim> optProblem(sys, cost, lin);
    optProblem.setInitialState(x_0);
    optProblem.setTimeHorizon(settings.T_);

    using DmsSolver_t = ct::optcon::DmsSolver<state_dim, control_dim, double, false>;

    std::shared_ptr<DmsSolver_t> dmsSolver(new DmsSolver_t(optProblem, settings));

    dmsSolver->solve();
    auto solutionPolicy = dmsSolver->getSolution();

    auto stateSolution = solutionPolicy.xSolution_;
    auto inputSolution = solutionPolicy.uSolution_;
    auto timeSolution = solutionPolicy.tSolution_;
}

}  // namespace optcon
}  // namespace ct

int main(int argc, char** argv)
{
    // using namespace ct::optcon::example;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
