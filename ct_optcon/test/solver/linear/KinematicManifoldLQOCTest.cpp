
#include <ct/optcon/optcon.h>

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

    const bool use_single_shooting = true;  // toggle between single and multiple shooting

    const size_t N = 17;
    const double dt = 0.1;

    const ManifoldState_t x0 = manif::SO3<double>(M_PI / 2, 0, 0);
    ct::core::DiscreteArray<ManifoldState_t> x_traj(N + 1, x0);  // init state trajectory, will be overwritten
    ct::core::DiscreteArray<ManifoldState_t::Tangent> b(
        N + 1, ManifoldState_t::Tangent::Zero());                             // defect traj, will be overwritten
    ct::core::DiscreteArray<ct::core::ControlVector<control_dim>> u_traj(N);  // init control traj
    for (size_t i = 0; i < N; i++)
        u_traj[i] = ct::core::ControlVector<control_dim>::Random() * 0.01;

    // choose a random initial state
    // TODO: numerical trouble for more aggressive distributions, since the approximation of the value function becomes really bad?
    for (size_t i = 1; i < N + 1; i++)
    {
        x_traj[i] = ManifoldState_t::Random();
    }

    // create instances of HPIPM and an unconstrained Gauss-Newton Riccati solver
    std::shared_ptr<LQOCSolver<ManifoldState_t, control_dim>> gnRiccatiSolver(
        new GNRiccatiSolver<ManifoldState_t, control_dim>);
    // std::shared_ptr<LQOCSolver<ManifoldState_t, control_dim>> hpipmSolver(
    //     new HPIPMInterface<ManifoldState_t, control_dim>);

    // store them, and identifying names, in a vectors
    std::vector<std::shared_ptr<LQOCSolver<ManifoldState_t, control_dim>>> lqocSolvers;
    lqocSolvers.push_back(gnRiccatiSolver);
    // lqocSolvers.push_back(hpipmSolver);
    std::vector<std::string> solverNames = {"Riccati", "hpipm"};

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
    Q_final.setZero();
    Q_final.diagonal() << 10000, 10000, 10000;
    Q.setZero();
    Q.diagonal() << 1, 1, 1;
    R.setZero();
    R.diagonal() << 1, 1, 1;
    ManifoldState_t x_final = manif::SO3<double>(0, 0, 0);
    std::cout << "desired final state: " << x_final << std::endl;
    ManifoldState_t x_nominal = x_final;
    ct::core::ControlVector<control_dim> u_nom = ct::core::ControlVector<control_dim>::Zero();
    std::shared_ptr<CostFunctionQuadratic<ManifoldState_t, control_dim>> costFunction(
        new CostFunctionQuadraticSimple<ManifoldState_t, control_dim>(Q, R, x_nominal, u_nom, x_final, Q_final));


    // integrate an initial state with the open-loop system to get initial trajectories
    ManifoldState_t x_curr;
    ManifoldState_t::Tangent dx;
    x_curr = x0;
    x_traj.front() = x0;
    std::cout << "integrate an random initial state with the unstable system" << std::endl;
    std::cout << std::setprecision(4) << "m: " << x_curr << "\t tan: " << x_curr.log() << std::endl;
    for (size_t i = 0; i < N; i++)
    {
        exampleSystem->computeControlledDynamics(x_traj[i], 0, u_traj[i], dx);
        x_curr = x_traj[i] + dx;
        if (use_single_shooting)
            x_traj[i + 1] = x_curr;
        b[i] = dx - x_traj[i + 1].rminus(x_traj[i]);
        std::cout << "b: " << b[i] << std::endl;
        std::cout << std::setprecision(4) << "m: " << x_curr << "\t tan: " << x_curr.log() << std::endl;
    }

    size_t nIter = 25;
    for (size_t iter = 0; iter < nIter; iter++)
    {
        // initialize the optimal control problems for both solvers
        problems[0]->setFromTimeInvariantLinearQuadraticProblem(x_traj, u_traj, *linearizer, *costFunction, b, dt);
        problems[1]->setFromTimeInvariantLinearQuadraticProblem(x_traj, u_traj, *linearizer, *costFunction, b, dt);


        // HACKY corrections // TODO: move somewhere meaningful
        for (size_t idx : {0})
        {
            // dynamics transportation
            for (size_t i = 0; i < N; i++)
            {
                auto l = x_traj[i + 1].rminus(x_traj[i]);
                auto l_adj = (l.exp()).adj();

                problems[idx]->Adj_x_[i + 1] = l_adj;  // parallel transport matrix / adjoint from stage k+1 to stage k

                if (idx == 0)  // make the corrections for the standard riccati solver
                {
                    problems[idx]->A_[i] = l_adj.transpose() * problems[idx]->A_[i];
                    problems[idx]->B_[i] = l_adj.transpose() * problems[idx]->B_[i];
                    problems[idx]->b_[i] = l_adj.transpose() * problems[idx]->b_[i];
                }
            }

            // set the problem pointers
            lqocSolvers[idx]->setProblem(problems[idx]);

            // allocate memory (if required)
            lqocSolvers[idx]->initializeAndAllocate();

            // solve the problems...
            lqocSolvers[idx]->solve();

            // postprocess data
            lqocSolvers[idx]->compute_lv();
            lqocSolvers[idx]->computeFeedbackMatrices();
            lqocSolvers[idx]->computeStatesAndControls();
        }

        // retrieve solutions from both solvers
        auto xSol_riccati = lqocSolvers[0]->getSolutionState();
        auto uSol_riccati = lqocSolvers[0]->getSolutionControl();
        ct::core::FeedbackArray<state_dim, control_dim> KSol_riccati = lqocSolvers[0]->getSolutionFeedback();
        ct::core::ControlVectorArray<control_dim> lv_sol_riccati = lqocSolvers[0]->get_lv();

        // auto xSol_hpipm = lqocSolvers[1]->getSolutionState();
        // auto uSol_hpipm = lqocSolvers[1]->getSolutionControl();
        // ct::core::FeedbackArray<state_dim, control_dim> KSol_hpipm = lqocSolvers[1]->getSolutionFeedback();
        // ct::core::ControlVectorArray<control_dim> lv_sol_hpipm = lqocSolvers[1]->get_lv();

        // compare the quantities
        //for (size_t i = 0; i < lv_sol_riccati.size(); i++)
        //{
        //    if ((lv_sol_riccati[i] - lv_sol_hpipm[i]).array().abs().maxCoeff() > 1e-8)
        //    {
        //        std::cout << std::setprecision(10) << lv_sol_riccati[i].transpose() << std::endl;
        //        std::cout << std::setprecision(10) << lv_sol_hpipm[i].transpose() << std::endl;
        //        throw std::runtime_error("lv solutions do not match");
        //    }
        //}
        //for (size_t i = 0; i < KSol_riccati.size(); i++)
        //{
        //    if (KSol_riccati[i].isApprox(KSol_hpipm[i], 1e-6) == false)
        //        throw std::runtime_error("K solutions do not match");
        //}
        //
        //for (size_t i = 0; i < uSol_riccati.size(); i++)
        //{
        //    if ((uSol_riccati[i] - uSol_hpipm[i]).array().abs().maxCoeff() > 1e-8)
        //    {
        //        std::cout << "for index " << i << std::endl;
        //        std::cout << std::setprecision(10) << uSol_riccati[i].transpose() << std::endl;
        //        std::cout << std::setprecision(10) << uSol_hpipm[i].transpose() << std::endl;
        //        throw std::runtime_error("u solutions do not match");
        //    }
        //}
        //for (size_t i = 0; i < xSol_riccati.size(); i++)
        //{
        //    if ((xSol_riccati[i] - xSol_hpipm[i]).coeffs().array().abs().maxCoeff() > 1e-8)
        //    {
        //        std::cout << xSol_riccati[i].transpose() << std::endl;
        //        std::cout << xSol_hpipm[i].transpose() << std::endl;
        //        throw std::runtime_error("x solutions do not match");
        //    }
        //}

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

        //std::cout << std::setprecision(4) << "Forward integrated closed-loop solution, iter :" << iter << std::endl;
        x_curr = x0;
        //std::cout << "m: " << x_curr << "\t tan: " << x_curr.log() << std::endl;
        ct::core::DiscreteArray<ManifoldState_t> x_traj_prev = x_traj;
        double d_cum_sum = 0;
        double dx_cum_sum = 0;
        double cost_sum = 0;
        for (size_t i = 0; i < N; i++)
        {
            dx.setZero();
            Eigen::Quaterniond old_rot(x_traj[i].w(), x_traj[i].x(), x_traj[i].y(), x_traj[i].z());

            if (use_single_shooting)
            {
                // TODO: some term is missing here;
                ManifoldState_t::Tangent x_err = x_traj[i].rminus(x_traj_prev[i]);
                u_traj[i] += lv_sol_riccati[i] + KSol_riccati[i] * (x_err /*- eucl. part here*/);
                exampleSystem->computeControlledDynamics(x_traj[i], i * dt, u_traj[i], dx);
                x_traj[i + 1] = x_traj[i] + dx;
            }
            else  // multiple shooting
            {
                u_traj[i] += uSol_riccati[i];
                x_traj[i + 1] += xSol_riccati[i + 1];
                exampleSystem->computeControlledDynamics(x_traj[i], i * dt, u_traj[i], dx);
            }

            Eigen::Quaterniond new_rot(x_traj[i + 1].w(), x_traj[i + 1].x(), x_traj[i + 1].y(), x_traj[i + 1].z());
            //std::cout << "m: " << x_traj[i + 1] << "\t dx: " << xSol_riccati[i + 1]
            //          << "\t -- rot diff norm(): " << old_rot.angularDistance(new_rot) << std::endl;

            // compute defect
            b[i] = dx - x_traj[i + 1].rminus(x_traj[i]);

            // compute update norms
            d_cum_sum += b[i].coeffs().norm();
            dx_cum_sum += xSol_riccati[i + 1].coeffs().norm();
            //std::cout << "b: " << b[i] << std::endl;
            // compute running cost
            costFunction->setCurrentStateAndControl(x_traj[i], u_traj[i], i * dt);
            cost_sum += costFunction->evaluateIntermediate();
        }

        // compute terminal cost
        costFunction->setCurrentStateAndControl(x_traj.back(), u_traj.back(), N * dt);
        cost_sum += costFunction->evaluateTerminal();

        std::cout << std::setprecision(10) << "d_norm: \t " << d_cum_sum << "\t dx_norm: \t" << dx_cum_sum
                  << " \t Jcost: " << cost_sum << std::endl;
    }  // end iter


    // save the x-trajectory to file
    std::vector<Eigen::Matrix3d> rot_traj;
    for(size_t i = 0; i<x_traj.size(); i++)
        rot_traj.push_back(x_traj[i].rotation());
    EigenFileExport::mat_to_file(EigenFileExport::CSVFormat(), "/tmp/rot_traj.csv", rot_traj);
    return 1;
}