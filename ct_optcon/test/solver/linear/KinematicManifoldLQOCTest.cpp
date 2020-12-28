
#include <ct/core/core.h>
#include <ct/optcon/costfunction/term/TermBase.hpp>
#include <ct/optcon/costfunction/term/TermBase-impl.hpp>
#include <ct/optcon/costfunction/CostFunction.hpp>
#include <ct/optcon/costfunction/CostFunction-impl.hpp>
#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>
#include <ct/optcon/costfunction/CostFunctionQuadratic-impl.hpp>
#include <ct/optcon/costfunction/CostFunctionQuadraticSimple.hpp>
#include <ct/optcon/costfunction/CostFunctionQuadraticSimple-impl.hpp>
#include <ct/optcon/problem/LQOCProblem.hpp>
#include <ct/optcon/problem/LQOCProblem-impl.hpp>
#include <ct/optcon/solver/lqp/GNRiccatiSolver.hpp>
#include <ct/optcon/solver/lqp/GNRiccatiSolver-impl.hpp>

using namespace ct::core;
using namespace ct::optcon;
const bool verbose = true;

using ManifoldState_t = ManifoldState<manif::SE3, manif::SE3Tangent>;
const size_t state_dim = ManifoldState_t::TangentDim;
const size_t control_dim = 6;

class DiscrSE3LTITestSystem final : public ct::core::ControlledSystem<ManifoldState_t, DISCRETE_TIME>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Base = ct::core::ControlledSystem<ManifoldState_t, DISCRETE_TIME>;

    DiscrSE3LTITestSystem() : Base(control_dim) {}
    virtual void computeControlledDynamics(const ManifoldState_t& m,
        const Time_t& n,
        const ct::core::ControlVectord& u,
        ManifoldState_t::Tangent& dx) override
    {
        if (u.size() != control_dim)
            throw std::runtime_error("control vector is not control_dim in computeControlledDynamics().");
        dx = u;
    }

    virtual DiscrSE3LTITestSystem* clone() const override { return new DiscrSE3LTITestSystem(); }
};

class DiscrSE3LinearSystem final : public ct::core::LinearSystem<ManifoldState_t, DISCRETE_TIME>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Base = ct::core::LinearSystem<ManifoldState_t, DISCRETE_TIME>;

    DiscrSE3LinearSystem() : Base(control_dim) {}
    virtual DiscrSE3LinearSystem* clone() const override { return new DiscrSE3LinearSystem(); }

    const ct::core::StateMatrix<state_dim>& getDerivativeState(const int t = 0.0) override { return A_; }
    const ct::core::StateControlMatrix<state_dim>& getDerivativeControl(const int t = 0.0) override { return B_; }

    // TODO: take-away - in a discrete system, A must not be zero but at least identity.
    ct::core::StateMatrix<state_dim> A_ = ct::core::StateMatrix<state_dim>::Identity(state_dim, state_dim);
    ct::core::StateControlMatrix<state_dim> B_ =
        ct::core::StateControlMatrix<state_dim>::Identity(state_dim, control_dim);
};

void testCoordinateTransformCost()
{
    using namespace manif;
    for (size_t i = 0; i < 3; i++)
    {
        SE3d X, Xf;
        X.setRandom();
        Xf.setRandom();
        Eigen::Matrix<double, 6, 6> Jl_tau, Jr_tau, Jl_zet, Jr_zet;
        auto tau = Xf.rminus(X, Jl_tau, Jr_tau);
        auto zet = X.rminus(Xf, Jl_zet, Jr_zet);
        auto Adj_tau = tau.exp().adj();
        auto Adj_zet = zet.exp().adj();
        // std::cout << Adj_tau.transpose() * Adj_tau << std::endl;
        auto tau_recon = -Adj_tau * zet;
        auto tau_recon_J = Jr_tau * zet;
        std::cout << "tau:" << tau << std::endl;
        std::cout << "-Adj_tau * zet:" << std::endl << tau_recon.transpose() << std::endl;
        std::cout << "Jr_tau * zet:" << std::endl << tau_recon_J.transpose() << std::endl;
        std::cout << "-Jl_zet * zet:" << std::endl << (-Jl_zet * zet).transpose() << std::endl;
        std::cout << "zet:" << zet << std::endl;
        auto zet_recon = -Adj_zet * tau;
        auto zet_recon_J = Jr_zet * tau;
        std::cout << "-Adj_zet * tau:" << std::endl << zet_recon.transpose() << std::endl;
        std::cout << "Jr_zet * tau:" << std::endl << zet_recon_J.transpose() << std::endl;
        std::cout << "-Jl_tau * tau:" << std::endl << (-Jl_tau * tau).transpose() << std::endl;
        // construct a random symmetric matrix Q
        Eigen::Matrix<double, 6, 6> Q, A;
        A.setRandom();
        Q = A + A.transpose();
        // these costs all evaluate the same scalar value, so that means, for the second-order term cost eval we are fine!
        // we note, however, that the hessian matrices themselves are not identical!!!!
        std::cout << tau.transpose() * Q * tau << std::endl;
        std::cout << zet.transpose() * Q * zet << std::endl;
        std::cout << tau.transpose() * Adj_zet.transpose() * Q * Adj_zet * tau << std::endl;
        std::cout << tau.transpose() * Jr_zet.transpose() * Q * Jr_zet * tau << std::endl;
        std::cout << std::endl;
        // .. but those matrices are not identical:
        // std::cout << Adj_zet.transpose() * Q * Adj_zet << std::endl;
        // std::cout << Jr_zet.transpose() * Q * Jr_zet << std::endl;
        std::cout << std::endl;
    }
}

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


void ocqpTest()
{
    std::cout << std::fixed;

    const bool use_single_shooting = true;  // toggle between single and multiple shooting

    const size_t N = 150;
    const double dt = 1.0;

    const ManifoldState_t x0 = manif::SE3<double>(1, 0, 0, 1, 0, 1);
    ct::core::DiscreteArray<ManifoldState_t> x_traj(N + 1, x0);  // init state trajectory, will be overwritten
    ct::core::DiscreteArray<ManifoldState_t::Tangent> b(
        N + 1, ManifoldState_t::Tangent::Zero());                 // defect traj, will be overwritten
    ct::core::DiscreteArray<ct::core::ControlVectord> u_traj(N);  // init control traj
    for (size_t i = 0; i < N; i++)
    {
        u_traj[i] =
            ct::core::ControlVectord::Random(control_dim) *
            0.1;  // TODO: can we push the value function to a local minimum if we add initial control inputs that drive the system "one time around" ?
    }

    for (size_t i = 1; i < N + 1; i++)
    {
        //x_traj[i] = ManifoldState_t::Random();
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
    std::shared_ptr<ct::core::ControlledSystem<ManifoldState_t, DISCRETE_TIME>> exampleSystem(
        new DiscrSE3LTITestSystem());
    std::shared_ptr<ct::core::LinearSystem<ManifoldState_t, DISCRETE_TIME>> linearSystem(new DiscrSE3LinearSystem());


    // create a cost function
    Eigen::Matrix<double, state_dim, state_dim> Q, Q_final;
    Eigen::Matrix<double, control_dim, control_dim> R;
    Q_final.setZero();
    Q.setZero();
    Q_final.diagonal() << 1000, 1000, 1000, 1000, 1000, 1000;
    // std::cout << Q_final.eigenvalues() << std::endl;
    // std::cout << Q_final << std::endl;
    Q.diagonal() << .1, .1, .1, .1, .1, .1;
    R.setZero();
    R.diagonal() << 1, 1, 1, 1, 1, 1;
    ManifoldState_t x_final = manif::SE3<double>(0, 0, 0, 0, 1, 0);
    std::cout << "desired final state: " << x_final << std::endl;
    ManifoldState_t x_nominal = x_final;
    ct::core::ControlVectord u_nom = ct::core::ControlVectord::Zero(control_dim);
    std::shared_ptr<CostFunctionQuadratic<ManifoldState_t, control_dim>> costFunction(
        new CostFunctionQuadraticSimple<ManifoldState_t, control_dim>(Q, R, x_nominal, u_nom, x_final, Q_final));


    // integrate an initial state with the open-loop system to get initial trajectories
    ManifoldState_t x_curr = x0;
    x_traj.front() = x0;
    ManifoldState_t::Tangent dx;
    //std::cout << "integrate an random initial state with the unstable system" << std::endl;
    //std::cout << std::setprecision(4) << "m: " << x_curr << "\t tan: " << x_curr.log() << std::endl;
    for (size_t i = 0; i < N; i++)
    {
        exampleSystem->computeControlledDynamics(x_traj[i], 0, u_traj[i], dx);
        x_curr = x_traj[i] + dx;
        if (use_single_shooting)
            x_traj[i + 1] = x_curr;
        b[i] = dx - x_traj[i + 1].rminus(x_traj[i]);
        // std::cout << "b: " << b[i] << std::endl;
        // std::cout << std::setprecision(4) << "m: " << x_curr << "\t tan: " << x_curr.log() << std::endl;
    }

    size_t nIter = 13;
    for (size_t iter = 0; iter < nIter; iter++)
    {
        // initialize the optimal control problems for both solvers
        problems[0]->setFromTimeInvariantLinearQuadraticProblem(x_traj, u_traj, *linearSystem, *costFunction, b, dt);
        problems[1]->setFromTimeInvariantLinearQuadraticProblem(x_traj, u_traj, *linearSystem, *costFunction, b, dt);

        // HACKY corrections // TODO: move somewhere meaningful
        for (size_t idx : {0})
        {
            // dynamics transportation
            for (size_t i = 0; i < N; i++)
            {
                Eigen::Matrix<double, state_dim, state_dim> Jl, Jr;
                auto l = x_traj[i + 1].rminus(x_traj[i], Jl, Jr);
                // auto l_adj = (l.exp()).adj();
                // auto between_adj = x_traj[i + 1].between(x_traj[i]).adj();

                auto m = Jl.transpose();  // transport matrix / adjoint from stage k+1 to stage k
                problems[idx]->Adj_x_[i + 1] =
                    m.inverse();  // TODO: could we find a different expression, e.g. something like -Jr?

                problems[idx]->A_[i] = m * problems[idx]->A_[i];
                problems[idx]->B_[i] = m * problems[idx]->B_[i];
                problems[idx]->b_[i] = m * problems[idx]->b_[i];
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
        ct::core::ControlVectorArray<double> lv_sol_riccati = lqocSolvers[0]->get_lv();

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
            //Eigen::Quaterniond old_rot(x_traj[i].quat().w(), x_traj[i].quat().x(), x_traj[i].quat().y(), x_traj[i].quat().z());

            if (use_single_shooting)
            {
                Eigen::Matrix<double, state_dim, state_dim> Jl, Jr;
                ManifoldState_t::Tangent x_err = x_traj[i].rminus(x_traj_prev[i], Jl, Jr);
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

            //Eigen::Quaterniond new_rot(
            //    x_traj[i + 1].quat().w(), x_traj[i + 1].quat().x(), x_traj[i + 1].quat().y(), x_traj[i + 1].quat().z());
            //std::cout << "m: " << x_traj[i + 1] << "\t dx: " << xSol_riccati[i + 1]
            //          << "\t -- rot diff norm(): " << old_rot.angularDistance(new_rot) << std::endl;

            // compute defect
            b[i] = dx - (x_traj[i + 1].rminus(x_traj[i]));

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

    std::cout << "the final trajectory: " << std::endl;

    // save the x-trajectory to file
    std::vector<Eigen::Matrix3d> rot_traj;
    std::vector<Eigen::Vector3d> trans_traj;
    for (size_t i = 0; i < x_traj.size(); i++)
    {
        std::cout << x_traj[i] << std::endl;
        rot_traj.push_back(x_traj[i].rotation());
        trans_traj.push_back(x_traj[i].translation());
    }
    EigenFileExport::mat_to_file(EigenFileExport::CSVFormat(), "/tmp/rot_traj.csv", rot_traj);
    EigenFileExport::mat_to_file(EigenFileExport::CSVFormat(), "/tmp/trans_traj.csv", trans_traj);
}


int main(int argc, char** argv)
{
    //testCoordinateTransformCost();
    ocqpTest();
    return 1;
}