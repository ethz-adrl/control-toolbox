/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * This unit test compares HPIPM and the custom GNRiccati solver using custom defined system
 * with state dimension 8 and control dimension 3.
 *
 * \warning This example is not intuitive. For a better introduction into the solver-framework,
 * visit the tutorial.
 */


using namespace ct::core;

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

class LinkedMasses : public LinearSystem<8, 3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const int state_dim = 8;
    static const int control_dim = 3;
    static const int pp = state_dim / 2;  // number of masses

    LinkedMasses()
    {
        A_.setZero();
        B_.setZero();


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


class LinkedMasses2 : public ControlledSystem<8, 3>
{
public:

	static const int state_dim = 8;
	static const int control_dim = 3;

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


TEST(HPIPMInterfaceTest, compareSolvers)
{
	const size_t state_dim = 8;
	const size_t control_dim = 3;

    int N = 5;
    double dt = 0.5;

    typedef ct::optcon::LQOCProblem<state_dim, control_dim> LQOCProblem_t;
    std::shared_ptr<LQOCProblem_t> lqocProblem_hpipm(new LQOCProblem_t(N));
    std::shared_ptr<LQOCProblem_t> lqocProblem_gnriccati(new LQOCProblem_t(N));

    // define an initial state
    StateVector<state_dim> x0;
    x0 << 2.5, 2.5, 0, 0, 0, 0, 0, 0;

    // define a desired terminal state
    StateVector<state_dim> stateOffset;
    stateOffset.setConstant(0.1);

    // define a nominal control
    ControlVector<control_dim> u0;
    u0.setConstant(-0.1);

    // define cost function matrices
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

    // initialize the linear quadratic optimal control problems
    lqocProblem_hpipm->setFromTimeInvariantLinearQuadraticProblem(
        x0, u0, discretizedSystem, costFunction, stateOffset, dt);
    lqocProblem_gnriccati->setFromTimeInvariantLinearQuadraticProblem(
        x0, u0, discretizedSystem, costFunction, stateOffset, dt);


    // create hpipm solver instance, set and solve problem
    ct::optcon::HPIPMInterface<state_dim, control_dim> hpipm;
    hpipm.setProblem(lqocProblem_hpipm);
    hpipm.solve();

    // create GNRiccati solver instance, set and solve problem
    ct::optcon::GNRiccatiSolver<state_dim, control_dim> gnriccati;
    gnriccati.setProblem(lqocProblem_gnriccati);
    gnriccati.solve();

    // retrieve solutions
    ct::core::StateVectorArray<state_dim> x_sol_hpipm = hpipm.getSolutionState();
    ct::core::StateVectorArray<state_dim> x_sol_gnrccati = gnriccati.getSolutionState();
    ct::core::ControlVectorArray<control_dim> u_sol_hpipm = hpipm.getSolutionControl();
    ct::core::ControlVectorArray<control_dim> u_sol_gnrccati = gnriccati.getSolutionControl();

    // asser that the solution sizes the same
    ASSERT_EQ(x_sol_hpipm.size(), x_sol_gnrccati.size());
    ASSERT_EQ(u_sol_hpipm.size(), u_sol_gnrccati.size());

    // assert that states are the same
    for (size_t i = 0; i < x_sol_hpipm.size(); i++)
    {
    	ASSERT_LT((x_sol_hpipm[i]-x_sol_gnrccati[i]).array().abs().maxCoeff(), 1e-6);
    }

    // assert that controls are the same
    for (size_t i = 0; i < u_sol_hpipm.size(); i++)
    {
    	ASSERT_LT((u_sol_hpipm[i]-u_sol_gnrccati[i]).array().abs().maxCoeff(), 1e-6);
    }
}
