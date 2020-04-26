
#include <ct/optcon/optcon.h>
#include <ct/optcon/solver/lqp/AugGNRiccatiSolver.hpp>
#include <ct/optcon/solver/lqp/AugGNRiccatiSolver-impl.hpp>

using namespace ct::core;
using namespace ct::optcon;


const bool verbose = true;

using ManifoldState_t = ManifoldState<manif::SO3, manif::SO3Tangent>;
const size_t state_dim = ManifoldState_t::TangentDim;
const size_t control_dim = 3;

class DiscrSO3LTITestSystem final : public ct::core::ControlledSystem<ManifoldState_t, control_dim, DISCRETE_TIME>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DiscrSO3LTITestSystem() {}
    virtual void computeControlledDynamics(const ManifoldState_t& m,
        const Time_t& n,
        const ct::core::ControlVector<control_dim>& u,
        ManifoldState_t::Tangent& dx) override
    {
        dx = u;
    }

    virtual DiscrSO3LTITestSystem* clone() const override { return new DiscrSO3LTITestSystem(); }
    /**
     * @brief the log operator is defined as expressing the tangent vector w.r.t. m_ref_
     */
    virtual ManifoldState_t::Tangent lift(const ManifoldState_t& m) override
    {
        //throw std::runtime_error("not impl");
        return ManifoldState_t::Tangent::Zero();  // the system stays where it is (combination with A = identity())
        /*return m.rminus(m);*/
    }
    // virtual ManifoldState_t retract(const ManifoldState_t::Tangent& t) override {
    //     throw std::runtime_error("not implemetned.");
    //     /*return m_ref_.rplus(t);*/ }
protected:
    //ManifoldState_t m_ref_;
};


// TODO: make this a unit test
void testParallelTransport()
{
    auto origin = manif::SE2<double>(0, 0, 0);
    Eigen::Matrix3d b;
    b.setIdentity();  // some basis, assumed to be expressed in the manifolds over which we iterate

    for (double theta = M_PI / 2; theta <= 2 * M_PI; theta += M_PI / 2)
    {
        std::cout << "case theta = " << theta << std::endl;
        auto m = manif::SE2<double>(0, 0, theta);
        Eigen::Matrix3d Jl, Jr;  // jacobians
        auto d_rminus = m.rminus(origin, Jl, Jr);
        auto adj_rminus = d_rminus.exp().adj();  // the adjoint for transportation to the origin

        auto d_between = m.between(origin);
        auto adj_between = d_rminus.exp().adj();

        // ASSERT_TRUE(adj_rminus.isApprox(adj_between));

        std::cout << adj_rminus * b << std::endl << std::endl;   // express basis b in origin-manifold
        std::cout << adj_between * b << std::endl << std::endl;  // express basis b in origin-manifold
        // std::cout << Jr * Jl * b << std::endl << std::endl;  // TODO: how does the Jac relate to the adjoint here?
        std::cout << std::endl;
    }

    // auto m_pi_2 = manif::SE2<double>(0, 0, M_PI / 2);
    // auto m_pi = manif::SE2<double>(0, 0, M_PI);
    // auto m_mpi_2 = manif::SE2<double>(0, 0, -M_PI / 2);
    //
    //
    // std::cout << m_0.adj() * b << std::endl << std::endl;
    // std::cout << m_pi_2.adj() * b << std::endl << std::endl;
    // std::cout << m_pi.adj() * b << std::endl << std::endl;
    // std::cout << m_mpi_2.adj() * b << std::endl << std::endl;
}

int main(int argc, char** argv)
{
    std::cout << std::fixed;

    const size_t N = 20*3;
    const double dt = 0.125 / 3.0;

    const ManifoldState_t x0 = manif::SO3<double>(M_PI, 0, M_PI);
    ct::core::DiscreteArray<ManifoldState_t> x_traj(N + 1, x0);  // init state trajectory, will be overwritten
    ct::core::DiscreteArray<ct::core::ControlVector<control_dim>> u_traj(N);  // init control traj
    for (size_t i = 0; i < N; i++)
        u_traj[i] = ct::core::ControlVector<control_dim>::Ones() * dt;


    // create instances of HPIPM and an unconstrained Gauss-Newton Riccati solver
    std::shared_ptr<LQOCSolver<ManifoldState_t, control_dim>> hpipmSolver(
        new HPIPMInterface<ManifoldState_t, control_dim>);
    std::shared_ptr<LQOCSolver<ManifoldState_t, control_dim>> gnRiccatiSolver(
        new AugGNRiccatiSolver<ManifoldState_t, control_dim>);

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
    std::shared_ptr<ct::core::ControlledSystem<ManifoldState_t, control_dim, DISCRETE_TIME>> exampleSystem(
        new DiscrSO3LTITestSystem());
    std::shared_ptr<ct::core::SystemLinearizer<ManifoldState_t, control_dim, DISCRETE_TIME>> linearizer(
        new ct::core::SystemLinearizer<ManifoldState_t, control_dim, DISCRETE_TIME>(exampleSystem));


    // create a cost function
    Eigen::Matrix<double, state_dim, state_dim> Q, Q_final;
    Eigen::Matrix<double, control_dim, control_dim> R;
    Q_final << 10000, 0, 0, 0, 10000, 0, 0, 0, 10000;
    //Q << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Q.setZero();
    R << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    ManifoldState_t x_final = manif::SO3<double>(0, 0, 0);
    std::cout << "desired final state: " << x_final << std::endl;
    ManifoldState_t x_nominal = x0;
    ct::core::ControlVector<control_dim> u_nom = ct::core::ControlVector<control_dim>::Zero();
    std::shared_ptr<CostFunctionQuadratic<ManifoldState_t, control_dim>> costFunction(
        new CostFunctionQuadraticSimple<ManifoldState_t, control_dim>(Q, R, x_nominal, u_nom, x_final, Q_final));

    ManifoldState_t::Tangent b;
    b.setZero();  // TODO why this?

    // integrate an initial state with the open-loop system to get initial trajectories
    ManifoldState_t x_curr;
    ManifoldState_t::Tangent dx;
    x_curr = x0;
    x_traj.front() = x0;
    std::cout << "integrate an random initial state with the unstable system" << std::endl;
    std::cout << std::setprecision(4) << "m: " << x_curr << "\t tan: " << x_curr.log() << std::endl;
    for (size_t i = 0; i < N; i++)
    {
        exampleSystem->computeControlledDynamics(x_curr, 0, u_traj[i], dx);
        x_curr = x_curr + dx;
        x_traj[i + 1] = x_curr;
        std::cout << std::setprecision(4) << "m: " << x_curr << "\t tan: " << x_curr.log() << std::endl;
    }

    size_t nIter = 6;
    for (size_t iter = 0; iter < nIter; iter++)
    {
        // initialize the optimal control problems for both solvers
        problems[0]->setFromTimeInvariantLinearQuadraticProblem(x_traj, u_traj, *linearizer, *costFunction, b, dt);
        problems[1]->setFromTimeInvariantLinearQuadraticProblem(x_traj, u_traj, *linearizer, *costFunction, b, dt);


        // HACKY corrections // TODO: move somewhere meaningful
        for (size_t idx : {0})  // for riccati solver only
        {
            // intermediate stages cost transportation
            for (size_t i = 0; i < N; i++)
            {
                auto e = x_nominal.rminus(x_traj[i]);
                // compute PT matrix w.r.t. current ref traj // TODO: clarifiy formulation of error in cost function
                auto e_adj = (e.exp()).adj();
                problems[idx]->Q_[i] = e_adj * problems[idx]->Q_[i] * e_adj.transpose();
                problems[idx]->qv_[i] = e_adj * problems[idx]->qv_[i];
            }
            // terminal stage transportation
            // TODO: clarifiy formulation of error in cost function
            auto e = x_final.rminus(x_traj[N]);  // compute PT matrix w.r.t. current ref traj
            auto e_adj = (e.exp()).adj();
            //std::cout << "cost adj" << std::endl << e_adj << std::endl;
            problems[idx]->Q_.back()  = /*e_adj * */  problems[idx]->Q_.back(); // * e_adj.transpose();
            problems[idx]->qv_.back() = /*e_adj * */  problems[idx]->qv_.back();

            // dynamics transportation
            for (size_t i = 0; i < N; i++)
            {
                // std::cout << "dyn transport matrices" << std::endl;
                Eigen::Matrix3d Jl, Jr;
                auto l = x_traj[i + 1].rminus(x_traj[i], Jl, Jr);  // TODO: is it the right way round?
                auto l_adj = (l.exp()).adj();
                //std::cout << "l_adj" << std::endl << l_adj << std::endl;
                //std::cout << std::endl;
                // problems[idx]->A_[i] = l_adj.transpose() * problems[idx]->A_[i];  // todo: are those the right
                // problems[idx]->B_[i] = l_adj.transpose() * problems[idx]->B_[i];  // TODO: are those the right
                problems[idx]->Acal_[i + 1] = l_adj;
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
        //std::cout << std::setprecision(4) << "dx solution from riccati solver and directly added state_traj, iter "
        //          << iter << std::endl;
        //ct::core::DiscreteArray<ManifoldState_t> x_solver_direct(N + 1);
        //for (size_t j = 0; j < xSol_riccati.size(); j++)
        //{
        //    x_solver_direct[j] = x_traj[j] + xSol_riccati[j];
        //    double angularDiff = 0;
        //    if (j > 0)
        //    {
        //        Eigen::Quaterniond old_rot(x_solver_direct[j - 1].w(), x_solver_direct[j - 1].x(),
        //            x_solver_direct[j - 1].y(), x_solver_direct[j - 1].z());
        //        Eigen::Quaterniond new_rot(
        //            x_solver_direct[j].w(), x_solver_direct[j].x(), x_solver_direct[j].y(), x_solver_direct[j].z());
        //        angularDiff = old_rot.angularDistance(new_rot);
        //    }
        //    std::cout << "m:" << x_solver_direct[j] << "\t dx:" << xSol_riccati[j].transpose()
        //              << "\t -- rot diff norm(): " << angularDiff << std::endl;
        //}

        std::cout << std::setprecision(4) << "Forward integrated closed-loop solution, iter :" << iter << std::endl;
        x_curr = x0;
        std::cout << "m: " << x_curr << "\t tan: " << x_curr.log() << std::endl;
        ct::core::DiscreteArray<ManifoldState_t> x_traj_prev = x_traj;
        for (size_t i = 0; i < N; i++)
        {
            ManifoldState_t::Tangent x_err = x_curr.rminus(x_traj_prev[i]);
            // TODO: some term is missing here;
            const ct::core::ControlVector<control_dim> u =
                u_traj[i] + lv_sol_riccati[i] + KSol_riccati[i] * (x_err /*- eucl. part here*/);
            u_traj[i] = u;

            exampleSystem->computeControlledDynamics(x_curr, i * dt, u, dx);
            Eigen::Quaterniond old_rot(x_curr.w(), x_curr.x(), x_curr.y(), x_curr.z());
            x_curr = x_curr + dx;
            Eigen::Quaterniond new_rot(x_curr.w(), x_curr.x(), x_curr.y(), x_curr.z());
            std::cout << "m: " << x_curr << "\t tan: " << x_curr.log()
                      << "\t -- rot diff norm(): " << old_rot.angularDistance(new_rot) << std::endl;

            x_traj[i + 1] = x_curr;
        }
        std::cout << std::endl << std::endl;
    }  // end iter
    return 1;
}