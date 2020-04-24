
#include <ct/optcon/optcon.h>

using namespace ct::core;
using namespace ct::optcon;


const bool verbose = true;

using ManifoldState_t = ManifoldState<manif::SO3, manif::SO3Tangent>;
const size_t state_dim = ManifoldState_t::TangentDim;
const size_t control_dim = 3;

class DiscrSO3LTITestSystem final : public LTISystem<ManifoldState_t, control_dim, DISCRETE_TIME>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DiscrSO3LTITestSystem(const Eigen::Matrix<double, state_dim, state_dim> A_test,
        const Eigen::Matrix<double, control_dim, control_dim> B_test,
        const ManifoldState_t& m_ref = ManifoldState_t::NeutralElement())
        : m_ref_(m_ref)
    {
        this->A() << A_test;
        this->B() << B_test;
    }

    /**
     * @brief the log operator is defined as expressing the tangent vector w.r.t. m_ref_
     */
    virtual ManifoldState_t::Tangent lift(const ManifoldState_t& m) override
    {
        return ManifoldState_t::Tangent::Zero();  // the system stays where it is (combination with A = identity())
        /*return m.rminus(m);*/
    }
    // virtual ManifoldState_t retract(const ManifoldState_t::Tangent& t) override {
    //     throw std::runtime_error("not implemetned.");
    //     /*return m_ref_.rplus(t);*/ }
protected:
    ManifoldState_t m_ref_;
};


int main(int argc, char** argv)
{
    std::cout << std::fixed;

    const size_t N = 5;
    const double dt = 0.25;

    const ManifoldState_t x0 = manif::SO3<double>(0, 0, 0);
    ct::core::DiscreteArray<ManifoldState_t> x_traj(N + 1, x0);  // init state trajectory, will be overwritten
    ct::core::DiscreteArray<ct::core::ControlVector<control_dim>> u_traj_init(N);  // init control traj
    for (size_t i = 0; i < N; i++)
        u_traj_init[i] = ct::core::ControlVector<control_dim>::Zero() * dt;


    // create instances of HPIPM and an unconstrained Gauss-Newton Riccati solver
    std::shared_ptr<LQOCSolver<ManifoldState_t, control_dim>> hpipmSolver(
        new HPIPMInterface<ManifoldState_t, control_dim>);
    std::shared_ptr<LQOCSolver<ManifoldState_t, control_dim>> gnRiccatiSolver(
        new GNRiccatiSolver<ManifoldState_t, control_dim>);

    // store them, and identifying names, in a vectors
    std::vector<std::shared_ptr<LQOCSolver<ManifoldState_t, control_dim>>> lqocSolvers;
    lqocSolvers.push_back(gnRiccatiSolver);
    lqocSolvers.push_back(hpipmSolver);
    std::vector<std::string> solverNames = {"Riccati", "HPIPM"};

    // create linear-quadratic optimal control problem containers
    std::vector<std::shared_ptr<LQOCProblem<ManifoldState_t, control_dim>>> problems;
    std::shared_ptr<LQOCProblem<ManifoldState_t, control_dim>> lqocProblem1(
        new LQOCProblem<ManifoldState_t, control_dim>(N));
    std::shared_ptr<LQOCProblem<ManifoldState_t, control_dim>> lqocProblem2(
        new LQOCProblem<ManifoldState_t, control_dim>(N));

    problems.push_back(lqocProblem1);
    problems.push_back(lqocProblem2);

    // create a discrete-time manifold system
    Eigen::Matrix<double, state_dim, state_dim> A = Eigen::Matrix<double, state_dim, state_dim>::Zero();
    Eigen::Matrix<double, control_dim, control_dim> B;
    A << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0;  // a slightly unstable system
    B << 1, 0, 0, 0, 1, 0, 0, 0, 1;        // direct action
    std::shared_ptr<LinearSystem<ManifoldState_t, control_dim, DISCRETE_TIME>> exampleSystem(
        new DiscrSO3LTITestSystem(A, B, x0));


    // create a cost function
    Eigen::Matrix<double, state_dim, state_dim> Q, Q_final;
    Eigen::Matrix<double, control_dim, control_dim> R;
    Q_final << 100, 0, 0, 0, 100, 0, 0, 0, 100;
    //Q << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Q.setZero();
    R << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    ManifoldState_t x_final = manif::SO3<double>(M_PI / 2, 0, 0);
    std::cout << "desired final state: " << x_final << std::endl;
    ManifoldState_t x_nominal = x0;
    ct::core::ControlVector<control_dim> u_nom = ct::core::ControlVector<control_dim>::Zero();
    std::shared_ptr<CostFunctionQuadratic<ManifoldState_t, control_dim>> costFunction(
        new CostFunctionQuadraticSimple<ManifoldState_t, control_dim>(Q, R, x_nominal, u_nom, x_final, Q_final));

    ManifoldState_t::Tangent b;
    b.setZero();  // todo why this?

    // integrate an initial state with the open-loop system to get initial trajectories
    ManifoldState_t x_curr;
    ManifoldState_t::Tangent dx;
    x_curr = x0;
    x_traj.front() = x0;
    std::cout << "integrate an random initial state with the unstable system" << std::endl;
    std::cout << "m: " << x_curr << "\t tan: " << x_curr.log() << std::endl;
    for (size_t i = 0; i < N; i++)
    {
        exampleSystem->computeControlledDynamics(x_curr, 0, u_traj_init[i], dx);
        x_curr = x_curr + dx;
        x_traj[i + 1] = x_curr;
        std::cout << "m: " << x_curr << "\t tan: " << x_curr.log() << std::endl;
    }


    // initialize the optimal control problems for both solvers
    problems[0]->setFromTimeInvariantLinearQuadraticProblem(x_traj, u_traj_init, *exampleSystem, *costFunction, b, dt);
    problems[1]->setFromTimeInvariantLinearQuadraticProblem(x_traj, u_traj_init, *exampleSystem, *costFunction, b, dt);


    // HACKY corrections // TODO: move somewhere meaningful
    for (size_t idx : {0, 1})  // for both solvers
    {
        // intermediate stages cost transportation
        for (size_t i = 0; i < N; i++)
        {
            Eigen::Matrix<double, state_dim, state_dim> Jl, Jr;
            auto e = x_nominal.rminus(x_traj[i], Jl, Jr);  // compute PT matrix w.r.t. current ref traj
            auto e_adj = (e.exp()).adj();
            problems[idx]->Q_[i] = e_adj.transpose() * problems[idx]->Q_[i] * e_adj;
            problems[idx]->qv_[i] = e_adj.transpose() * problems[idx]->qv_[i];
        }
        // terminal stage transportation
        Eigen::Matrix<double, state_dim, state_dim> Jl, Jr;
        auto e = x_final.rminus(x_traj[N], Jl, Jr);  // compute PT matrix w.r.t. current ref traj
        auto e_adj = (e.exp()).adj();
        std::cout << "cost adj" << std::endl << e_adj << std::endl;
        std::cout << "cost Jr" << std::endl << Jr << std::endl;
        problems[idx]->Q_.back() = e_adj.transpose() * problems[idx]->Q_.back() * e_adj;
        problems[idx]->qv_.back() = e_adj.transpose() * problems[idx]->qv_.back();

        // dynamics transportation
        for (size_t i = 0; i < N; i++)
        {
            // std::cout << "dyn transport matrices" << std::endl;
            Eigen::Matrix<double, state_dim, state_dim> Jl, Jr;

            auto l = x_traj[i + 1].rminus(x_traj[i], Jl, Jr);  // TODO: is it the right way round?
            auto l_adj = (l.exp()).adj();
            //std::cout << "Jl" << std::endl << Jl << std::endl;
            //std::cout << "Jr" << std::endl << Jr << std::endl;
            //std::cout << "l_adj" << std::endl << l_adj << std::endl;
            //std::cout << std::endl;
            problems[idx]->A_[i] = l_adj.transpose() * problems[idx]->A_[i];  // todo: are those the right
            problems[idx]->B_[i] = l_adj.transpose() * problems[idx]->B_[i];  // TODO: are those the right
        }


        // set the problem pointers
        lqocSolvers[idx]->setProblem(problems[idx]);

        // allocate memory (if required)
        lqocSolvers[idx]->initializeAndAllocate();

        // solve the problems...
        lqocSolvers[idx]->solve();

        // postprocess data
        lqocSolvers[idx]->computeStatesAndControls();
        lqocSolvers[idx]->computeFeedbackMatrices();
        lqocSolvers[idx]->compute_lv();
    }

    // retrieve solutions from both solvers
    auto xSol_riccati = lqocSolvers[0]->getSolutionState();
    auto uSol_riccati = lqocSolvers[0]->getSolutionControl();
    ct::core::FeedbackArray<state_dim, control_dim> KSol_riccati = lqocSolvers[0]->getSolutionFeedback();
    ct::core::ControlVectorArray<control_dim> lv_sol_riccati = lqocSolvers[0]->get_lv();
    auto xSol_hpipm = lqocSolvers[1]->getSolutionState();
    auto uSol_hpipm = lqocSolvers[1]->getSolutionControl();
    ct::core::FeedbackArray<state_dim, control_dim> KSol_hpipm = lqocSolvers[1]->getSolutionFeedback();
    ct::core::ControlVectorArray<control_dim> lv_sol_hpipm = lqocSolvers[1]->get_lv();

    std::cout << std::endl << std::endl;
    std::cout << std::setprecision(3) << "Forward integrated closed-loop solution:" << std::endl;
    x_curr = x0;
    std::cout << "m init: " << x_curr << std::endl;
    for (size_t i = 0; i < N; i++)
    {
        Eigen::Matrix<double, state_dim, state_dim> j1, j2;
        auto x_err = x_curr.rminus(x_traj[i], j1, j2);
        // std::cout << "lv_sol_riccati[i]: " << lv_sol_riccati[i].transpose() << std::endl;
        // std::cout << "uSol_riccati[i]: " << uSol_riccati[i].transpose() << std::endl;
        // std::cout << "KSol_riccati: " << std::endl << KSol_riccati[i] << std::endl;
        // TODO: some term is missing here;
        auto u = u_traj_init[i] + lv_sol_riccati[i] + KSol_riccati[i] * (x_err /*- eucl. part here*/);
        exampleSystem->computeControlledDynamics(x_curr, i * dt, u, dx);
        Eigen::Quaterniond old_rot(x_curr.w(), x_curr.x(), x_curr.y(), x_curr.z());
        x_curr = x_curr + dx;
        Eigen::Quaterniond new_rot(x_curr.w(), x_curr.x(), x_curr.y(), x_curr.z());
        std::cout << "m: " << x_curr << "\t -- rot diff norm(): " << old_rot.angularDistance(new_rot) << std::endl;
    }
    std::cout << std::endl << std::endl;

    std::cout << std::setprecision(3) << "dx solution from riccati solver and directly added state_traj" << std::endl;
    ct::core::DiscreteArray<ManifoldState_t> x_solver_direct(N + 1);
    for (size_t j = 0; j < xSol_riccati.size(); j++)
    {
        x_solver_direct[j] = x_traj[j] + xSol_riccati[j];
        double angularDiff = 0;
        if (j > 0)
        {
            Eigen::Quaterniond old_rot(x_solver_direct[j - 1].w(), x_solver_direct[j - 1].x(),
                x_solver_direct[j - 1].y(), x_solver_direct[j - 1].z());
            Eigen::Quaterniond new_rot(
                x_solver_direct[j].w(), x_solver_direct[j].x(), x_solver_direct[j].y(), x_solver_direct[j].z());
            angularDiff = old_rot.angularDistance(new_rot);
        }
        std::cout << "\t m:" << x_solver_direct[j] << "\t dx:" << xSol_riccati[j].transpose()
                  << "\t -- rot diff norm(): " << angularDiff << std::endl;
    }
    return 1;
}