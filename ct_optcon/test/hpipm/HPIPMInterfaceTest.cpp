/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * \warning This example is not intuitive. For a better introduction into the solver-framework,
 * visit the tutorial.
 */

#include <ct/optcon/optcon.h>

using namespace ct::core;

static const size_t state_dim = 8;
static const size_t control_dim = 3;

void dmcopy(int row, int col, double *A, int lda, double *B, int ldb)
{
    int i, j;
    for (j = 0; j < col; j++)
    {
        for (i = 0; i < row; i++)
        {
            B[i + j * ldb] = A[i + j * lda];
        }
    }
}

class LinkedMasses : public LinearSystem<state_dim, control_dim>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LinkedMasses()
    {
        A_.setZero();
        B_.setZero();

        static const int pp = state_dim / 2;  // number of masses

        Eigen::Matrix<double, pp, pp> TEigen;
        TEigen.setZero();

        double *T = TEigen.data();
        int ii;
        for (ii = 0; ii < pp; ii++)
            T[ii * (pp + 1)] = -2;
        for (ii = 0; ii < pp - 1; ii++)
            T[ii * (pp + 1) + 1] = 1;
        for (ii = 1; ii < pp; ii++)
            T[ii * (pp + 1) - 1] = 1;

        Eigen::Matrix<double, pp, pp> ZEigen;
        ZEigen.setZero();
        double *Z = ZEigen.data();

        Eigen::Matrix<double, pp, pp> IEigen;
        IEigen.setIdentity();
        double *I = IEigen.data();

        double *Ac = A_.data();
        dmcopy(pp, pp, Z, pp, Ac, state_dim);
        dmcopy(pp, pp, T, pp, Ac + pp, state_dim);
        dmcopy(pp, pp, I, pp, Ac + pp * state_dim, state_dim);
        dmcopy(pp, pp, Z, pp, Ac + pp * (state_dim + 1), state_dim);

        Eigen::Matrix<double, control_dim, control_dim> InuEigen;
        InuEigen.setIdentity();
        double *Inu = InuEigen.data();

        double *Bc = B_.data();
        dmcopy(control_dim, control_dim, Inu, control_dim, Bc + pp, state_dim);
    }


    const state_matrix_t &getDerivativeState(const StateVector<state_dim> &x,
        const ControlVector<control_dim> &u,
        const double t = 0.0) override
    {
        return A_;
    }

    const state_control_matrix_t &getDerivativeControl(const StateVector<state_dim> &x,
        const ControlVector<control_dim> &u,
        const double t = 0.0) override
    {
        return B_;
    }

    LinkedMasses *clone() const override { return new LinkedMasses(); };
private:
    state_matrix_t A_;
    state_control_matrix_t B_;
};


class LinkedMasses2 : public ControlledSystem<state_dim, control_dim>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LinkedMasses2()
    {
        A_.setZero();
        B_.setZero();
        b_.setZero();

        b_ << 0.145798, 0.150018, 0.150018, 0.145798, 0.245798, 0.200018, 0.200018, 0.245798;

        static const int pp = state_dim / 2;  // number of masses

        Eigen::Matrix<double, pp, pp> TEigen;
        TEigen.setZero();

        double *T = TEigen.data();
        int ii;
        for (ii = 0; ii < pp; ii++)
            T[ii * (pp + 1)] = -2;
        for (ii = 0; ii < pp - 1; ii++)
            T[ii * (pp + 1) + 1] = 1;
        for (ii = 1; ii < pp; ii++)
            T[ii * (pp + 1) - 1] = 1;

        Eigen::Matrix<double, pp, pp> ZEigen;
        ZEigen.setZero();
        double *Z = ZEigen.data();

        Eigen::Matrix<double, pp, pp> IEigen;
        IEigen.setIdentity();
        double *I = IEigen.data();

        double *Ac = A_.data();
        dmcopy(pp, pp, Z, pp, Ac, state_dim);
        dmcopy(pp, pp, T, pp, Ac + pp, state_dim);
        dmcopy(pp, pp, I, pp, Ac + pp * state_dim, state_dim);
        dmcopy(pp, pp, Z, pp, Ac + pp * (state_dim + 1), state_dim);

        Eigen::Matrix<double, control_dim, control_dim> InuEigen;
        InuEigen.setIdentity();
        double *Inu = InuEigen.data();

        double *Bc = B_.data();
        dmcopy(control_dim, control_dim, Inu, control_dim, Bc + pp, state_dim);
    }

    LinkedMasses2 *clone() const override { return new LinkedMasses2(); };
    void computeControlledDynamics(const ct::core::StateVector<state_dim> &state,
        const double &t,
        const ct::core::ControlVector<control_dim> &control,
        ct::core::StateVector<state_dim> &derivative) override
    {
        derivative = A_ * state + B_ * control + b_;
    }

private:
    ct::core::StateMatrix<state_dim> A_;
    ct::core::StateControlMatrix<state_dim, control_dim> B_;
    ct::core::StateVector<state_dim> b_;
};

void compareRiccatiApproach();  // forward declaration of comparison method


int main(int argc, char *argv[])
{
    int N = 5;
    double dt = 0.5;

    typedef ct::optcon::LQOCProblem<state_dim, control_dim> LQOCProblem_t;
    std::shared_ptr<LQOCProblem_t> lqocProblem(new LQOCProblem_t(N));

    // define an initial state
    StateVector<state_dim> x0;
    x0 << 2.5, 2.5, 0, 0, 0, 0, 0, 0;

    // define a desired terminal state
    StateVector<state_dim> stateOffset;
    stateOffset.setConstant(0.1);

    // define a nominal control
    ControlVector<control_dim> u0;
    u0.setConstant(-0.1);


    StateMatrix<state_dim> Q;
    Q.setIdentity();
    Q *= 2.0;
    ControlMatrix<control_dim> R;
    R.setIdentity();
    R *= 2 * 2.0;

    // create a cost function
    ct::optcon::CostFunctionQuadraticSimple<state_dim, control_dim> costFunction(
        Q, R, -stateOffset, u0, -stateOffset, Q);

    // create a continuous-time example system and discretize it
    std::shared_ptr<ct::core::LinearSystem<state_dim, control_dim>> system(new LinkedMasses());
    ct::core::SensitivityApproximation<state_dim, control_dim> discretizedSystem(
        dt, system, ct::optcon::NLOptConSettings::APPROXIMATION::MATRIX_EXPONENTIAL);

    // initialize the linear quadratic optimal control problem
    lqocProblem->setFromTimeInvariantLinearQuadraticProblem(x0, u0, discretizedSystem, costFunction, stateOffset, dt);


    // create hpipm solver instance, set and solve problem
    ct::optcon::HPIPMInterface<state_dim, control_dim> hpipm;
    hpipm.setProblem(lqocProblem);
    hpipm.solve();

    ct::core::StateVectorArray<state_dim> x_sol = hpipm.getSolutionState();

    for (size_t i = 0; i < x_sol.size(); i++)
        std::cout << x_sol[i].transpose() << std::endl;

    // todo print solution

    //    interface.changeTimeHorizon(N);
    //    interface.solveLinearProblem(x0, system, costFunction, stateOffset, dt);
    //    interface.printSolution();

    //    std::cout << std::endl << std::endl << std::endl;
    //    std::cout << "TEST GNMS!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl << std::endl << std::endl;
    //
    //    testGNMS();

    compareRiccatiApproach();

    return 1;
}

//
void compareRiccatiApproach()
{
    typedef ct::optcon::NLOptConSolver<state_dim, control_dim, state_dim / 2, state_dim / 2> NLOptConSolver;

    // define an initial state
    StateVector<state_dim> x_0;
    x_0 << 2.5, 2.5, 0, 0, 0, 0, 0, 0;

    //    // define a nominal control
    //    ControlVector<control_dim> u0;
    //    u0.setConstant(-0.1);


    ct::optcon::NLOptConSettings gnms_settings;
    gnms_settings.dt = 0.5;
    gnms_settings.K_sim = 1;
    gnms_settings.integrator = ct::core::IntegrationType::EULERCT;
    gnms_settings.discretization = ct::optcon::NLOptConSettings::APPROXIMATION::MATRIX_EXPONENTIAL;
    gnms_settings.max_iterations = 1;


    std::shared_ptr<ControlledSystem<state_dim, control_dim>> nonlinearSystem(new LinkedMasses2);
    //std::shared_ptr<LinearSystem<state_dim, control_dim> > analyticLinearSystem(new ct::core::SystemLinearizer<state_dim,control_dim>(nonlinearSystem));
    std::shared_ptr<LinearSystem<state_dim, control_dim>> analyticLinearSystem(new LinkedMasses);

    StateMatrix<state_dim> Q;
    Q.setIdentity();
    Q *= 2.0;
    ControlMatrix<control_dim> R;
    R.setIdentity();
    R *= 2 * 2.0;


    StateVector<state_dim> stateOffset;
    stateOffset.setConstant(0.1);

    ControlVector<control_dim> uNom;
    uNom.setConstant(-0.1);


    std::shared_ptr<ct::optcon::CostFunctionQuadratic<state_dim, control_dim>> costFunction(
        new ct::optcon::CostFunctionQuadraticSimple<state_dim, control_dim>(
            Q, R, -stateOffset, uNom, -stateOffset, Q * gnms_settings.dt));

    // times
    int N = 5;
    ct::core::Time tf = 5.0 * gnms_settings.dt;

    // provide initial guess
    ControlVectorArray<control_dim> u0(N, ControlVector<control_dim>::Zero());
    StateVectorArray<state_dim> x0(N + 1, x_0);
    FeedbackArray<state_dim, control_dim> u0_fb(N, FeedbackMatrix<state_dim, control_dim>::Zero());

    NLOptConSolver::Policy_t initController(x0, u0, u0_fb, gnms_settings.dt);

    ct::optcon::OptConProblem<state_dim, control_dim> optConProblem(
        tf, x0[0], nonlinearSystem, costFunction, analyticLinearSystem);


    NLOptConSolver gnms(optConProblem, gnms_settings);


    gnms.configure(gnms_settings);
    gnms.setInitialGuess(initController);

    std::cout << "running gnms solver" << std::endl;

    gnms.runIteration();

    NLOptConSolver::Policy_t solution = gnms.getSolution();

    StateVectorArray<state_dim> x_sol = solution.x_ref();

    for (size_t i = 0; i < x_sol.size(); i++)
        std::cout << x_sol[i].transpose() << std::endl;
}
