
#include <ct/optcon/optcon.h>

using namespace ct::core;
using namespace ct::optcon;
const bool verbose = true;


template <typename POS_MAN, typename VEL_MAN, typename POS_TANGENT, typename VEL_TANGENT>
class CompositeManifold;  // forward declaration, do not delete

template <typename POS_MAN, typename VEL_MAN, typename POS_TANGENT, typename VEL_TANGENT>
class CompositeManifoldTangent
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr size_t DoF = POS_TANGENT::DoF + VEL_TANGENT::DoF;

    using Scalar = double;
    using Jacobian = Eigen::Matrix<Scalar, DoF, DoF>;
    using OptJacobianRef = tl::optional<Eigen::Ref<Jacobian>>;

    CompositeManifoldTangent() : t_pos_(POS_TANGENT::Zero()), t_vel_(VEL_TANGENT::Zero()) { }
    CompositeManifoldTangent(const POS_TANGENT& p, const VEL_TANGENT& v) : t_pos_(p), t_vel_(v)
    {
    }

    template <typename _EigenDerived>
    CompositeManifoldTangent(const Eigen::MatrixBase<_EigenDerived>& v)
        : t_pos_(v.template head<POS_TANGENT::DoF>()), t_vel_(v.template tail<VEL_TANGENT::DoF>())
    {
        // TODO a dimension assert would not hurt
    }

    CompositeManifoldTangent(const CompositeManifoldTangent& arg)
        : t_pos_(arg.t_pos_), t_vel_(arg.t_vel_)
    {
    }

    static CompositeManifoldTangent Zero()
    {
        return CompositeManifoldTangent(POS_TANGENT::Zero(), VEL_TANGENT::Zero());
    }
    static CompositeManifoldTangent Random()
    {
        return CompositeManifoldTangent(POS_TANGENT::Random(), VEL_TANGENT::Random());
    }

    void setRandom()
    {
        t_pos_.setRandom();
        t_vel_.setRandom();
    }
    void setZero()
    {
        t_pos_.setZero();
        t_vel_.setZero();
    }

    // TODO: Here is the problem, trouble as soon as it gets passed by value.
    const Eigen::Matrix<Scalar, DoF, 1>& coeffs() const
    {
        Eigen::Matrix<Scalar, DoF, 1> coeffs_temp;
        coeffs_temp.template head<3>() = t_pos_.coeffs();
        coeffs_temp.template tail<3>() = t_vel_.coeffs();

        const_cast<Eigen::Matrix<Scalar, DoF, 1>&>(coeffs_) = coeffs_temp;

        return coeffs_;
    }


    Scalar& operator()(const int k) { return k < POS_TANGENT::DoF ? t_pos_.coeffs()(k) : t_vel_.coeffs()(k - 3); }
    const Scalar& operator()(const int k) const
    {
        return k < POS_TANGENT::DoF ? t_pos_.coeffs()(k) : t_vel_.coeffs()(k - 3);
    }

    CompositeManifoldTangent operator-(const CompositeManifoldTangent& tb) const
    {
        CompositeManifoldTangent res;
        res.set_pos(pos() - tb.pos());
        res.set_vel(vel() - tb.vel());
        return res;
    }

    CompositeManifoldTangent operator+(const CompositeManifoldTangent& tb) const
    {
        CompositeManifoldTangent res;
        res.set_pos(pos() + tb.pos());
        res.set_vel(vel() + tb.vel());
        return res;
    }
    CompositeManifoldTangent operator+=(const CompositeManifoldTangent& tb)
    {
        set_pos(pos() + tb.pos());
        set_vel(vel() + tb.vel());
        return *this;
    }

    CompositeManifold<POS_MAN, VEL_MAN, POS_TANGENT, VEL_TANGENT> exp(OptJacobianRef J_m_t = OptJacobianRef{}) const
    {
        if (J_m_t)
        {
            throw std::runtime_error("J-m-t in tangent exp not impl yet.");
        }

        CompositeManifold<POS_MAN, VEL_MAN, POS_TANGENT, VEL_TANGENT> c(
            this->t_pos_.exp(), this->t_pos_.exp().adj().transpose() * this->t_vel_.coeffs());
        return c;
    }

    auto transpose() const { return coeffs().transpose(); }
    const POS_TANGENT& pos() const { return t_pos_; }
    void set_pos(const POS_TANGENT& p)
    {
        t_pos_ = p;
    }
    void set_vel(const VEL_TANGENT& v)
    {
        t_vel_ = v;
    }
    const VEL_TANGENT& vel() const { return t_vel_; }
protected:
    POS_TANGENT t_pos_;
    VEL_TANGENT t_vel_;
    Eigen::Matrix<Scalar, DoF, 1> coeffs_;  // is now used only in one function to return a ref
};

template <typename _Stream, typename t1, typename t2, typename t3, typename t4>
_Stream& operator<<(_Stream& s, const CompositeManifoldTangent<t1, t2, t3, t4>& t)
{
    s << t.coeffs().transpose();
    return s;
}

template <class _EigenDerived, typename t1, typename t2, typename t3, typename t4>
auto operator*(const Eigen::MatrixBase<_EigenDerived>& J, const CompositeManifoldTangent<t1, t2, t3, t4>& t)
{
    return J * t.coeffs();
}

template <typename t1, typename t2, typename t3, typename t4>
auto operator*(const CompositeManifoldTangent<t1, t2, t3, t4>& t, const double& s)
{
    return t.coeffs() * s;
}

template <class _EigenDerived, typename t1, typename t2, typename t3, typename t4>
auto operator+(const Eigen::MatrixBase<_EigenDerived>& tb, const CompositeManifoldTangent<t1, t2, t3, t4>& t)
{
    CompositeManifoldTangent<t1, t2, t3, t4> res;
    CompositeManifoldTangent<t1, t2, t3, t4> rhs(tb);
    res.set_pos(t.pos() + rhs.pos());
    res.set_vel(t.vel() + rhs.vel());
    return res;
}


template <typename POS_MAN, typename VEL_MAN, typename POS_TANGENT, typename VEL_TANGENT>
class CompositeManifold
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Tangent = CompositeManifoldTangent<POS_MAN, VEL_MAN, POS_TANGENT, VEL_TANGENT>;
    static constexpr size_t TangentDim = Tangent::DoF;
    static constexpr size_t Dim = POS_MAN::Dim + VEL_MAN::Dim + 1;  // TODO: this is hacky

    using Scalar = double;
    using Jacobian = Eigen::Matrix<Scalar, TangentDim, TangentDim>;
    using OptJacobianRef = tl::optional<Eigen::Ref<Jacobian>>;

    // get ManifoldState templated on a different scalar from this expression
    template <typename OtherScalar>
    using RedefineScalar = CompositeManifold<POS_MAN, VEL_MAN, POS_TANGENT, VEL_TANGENT>;

    CompositeManifold() : m_pos_(POS_MAN::Identity()), m_vel_(VEL_MAN::Identity()) {}
    CompositeManifold(const POS_MAN& p, const VEL_MAN& v) : m_pos_(p), m_vel_(v) {}
    CompositeManifold(const CompositeManifold& arg) : m_pos_(arg.m_pos_), m_vel_(arg.m_vel_) {}
    static CompositeManifold NeutralElement() { return CompositeManifold::Identity(); }
    static CompositeManifold Identity() { return CompositeManifold(POS_MAN::Identity(), VEL_MAN::Identity()); }
    static CompositeManifold Random() { return CompositeManifold(POS_MAN::Random(), VEL_MAN::Random()); }
    void setIdentity()
    {
        m_pos_.setIdentity();
        m_vel_.setIdentity();
    }

    const Eigen::Matrix<Scalar, Dim, 1> coeffs() const  // TODO: return by value is bad
    {
        Eigen::Matrix<Scalar, Dim, 1> c;
        c << m_pos_.coeffs(), m_vel_.coeffs();
        return c;
    }

    CompositeManifold operator+(const Tangent& t) const { return rplus(t); }
    //CompositeManifold operator+=(const Tangent& t)
    //{
    //    *this = this->rplus(t);
    //    return *this;  // todo test
    //}

    /**
     * @brief right plus for composite manifold
     * NOTE: the operator also has to parallel transport velocities to the new tangent plane.
     * Since for rplus, the rhs is expressed in the tangent space of the lhs, we choose: "first add, then transport"
     */
    CompositeManifold rplus(const Tangent& t, OptJacobianRef Jl = {}, OptJacobianRef Jr = {}) const
    {
        typename POS_MAN::Jacobian pJl, pJr;
        typename VEL_MAN::Jacobian vJl, vJr;

        VEL_MAN v_new_local = vel().rplus(t.vel(), vJl, vJr);

        VEL_MAN v_new_transported(t.pos().exp().adj().transpose() * v_new_local.coeffs());

        CompositeManifold c;
        c.vel() = v_new_transported;
        //c.vel() = c.vel().rplus(t.vel(), vJl, vJr);
        c.pos() = pos().rplus(t.pos(), pJl, pJr);  // todo make jacobians optional

        if (Jl)
        {
            Jl->setZero();
            (*Jl).template topLeftCorner<POS_MAN::DoF, POS_MAN::DoF>() = pJl;
            (*Jl).template bottomRightCorner<VEL_MAN::DoF, VEL_MAN::DoF>() = vJl;
        }

        if (Jr)
        {
            Jr->setZero();
            (*Jr).template topLeftCorner<POS_MAN::DoF, POS_MAN::DoF>() = pJr;
            (*Jr).template bottomRightCorner<VEL_MAN::DoF, VEL_MAN::DoF>() =
                vJr;  // TODO: what will the velocity jacobians become?
        }

        return c;
    }


    /**
     * @brief right minus for composite manifold
     * NOTE: the operator also has to express velocities in the rhs tangent plane.
     * Since for rminus, the resulting difference is expressed in the tangent space of the rhs, we choose: "first transport, then substract"
     */
    Tangent rminus(const CompositeManifold& rhs, OptJacobianRef Jl = {}, OptJacobianRef Jr = {}) const
    {
        typename POS_MAN::Jacobian pJl, pJr;
        typename VEL_MAN::Jacobian vJl, vJr;

        Tangent t;
        t.set_pos(pos().rminus(rhs.pos(), pJl, pJr));  // todo make jacobians optional
        VEL_MAN v_lhs_transported(t.pos().exp().adj() * vel().coeffs());

        VEL_TANGENT v_new_local = v_lhs_transported.rminus(rhs.vel(), vJl, vJr);
        t.set_vel(v_new_local);

        if (Jl)
        {
            Jl->setIdentity();
            (*Jl).template topLeftCorner<POS_MAN::DoF, POS_MAN::DoF>() = pJl;
            (*Jl).template bottomRightCorner<VEL_MAN::DoF, VEL_MAN::DoF>() = vJl;
        }

        if (Jr)
        {
            Jr->setIdentity();
            (*Jr).template topLeftCorner<POS_MAN::DoF, POS_MAN::DoF>() = pJr;
            (*Jr).template bottomRightCorner<VEL_MAN::DoF, VEL_MAN::DoF>() = vJr;
        }

        return t;
    }


    Tangent log(OptJacobianRef J_t_m = {}) const
    {
        if (J_t_m)
            throw std::runtime_error("J_t_m compuation not defined.");

        return rminus(CompositeManifold::Identity(), J_t_m);
    }

    Jacobian adj() const
    {
        Jacobian J = Jacobian::Zero();
        J.template topLeftCorner<POS_MAN::DoF, POS_MAN::DoF>() = m_pos_.adj();
        J.template bottomRightCorner<VEL_MAN::DoF, VEL_MAN::DoF>() = m_pos_.adj();
        return J;
    }

    POS_MAN& pos() { return m_pos_; }
    const POS_MAN& pos() const { return m_pos_; }
    VEL_MAN& vel() { return m_vel_; }
    const VEL_MAN& vel() const { return m_vel_; }
protected:
    POS_MAN m_pos_;
    VEL_MAN m_vel_;
};
template <typename _Stream, typename t1, typename t2, typename t3, typename t4>
_Stream& operator<<(_Stream& s, const CompositeManifold<t1, t2, t3, t4>& m)
{
    s << m.coeffs().transpose();
    return s;
}


using ManifoldState_t = CompositeManifold<manif::SO3d, manif::R3d, manif::SO3Tangentd, manif::R3Tangentd>;
const size_t state_dim = ManifoldState_t::TangentDim;
const size_t control_dim = 3;


class DiscrSO3LTITestSystem final : public ct::core::ControlledSystem<ManifoldState_t, control_dim, DISCRETE_TIME>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DiscrSO3LTITestSystem(double dt) : dt_(dt) {}
    virtual void computeControlledDynamics(const ManifoldState_t& m,
        const Time_t& n,
        const ct::core::ControlVector<control_dim>& u,
        ManifoldState_t::Tangent& dx) override
    {
        dx.setZero();
        dx.set_vel(dt_ * u);                 // velocity increment
        dx.set_pos(dt_ * m.vel().coeffs());  // position increment
    }

    virtual DiscrSO3LTITestSystem* clone() const override { return new DiscrSO3LTITestSystem(*this); }
    /**
     * @brief the log operator is defined as expressing the tangent vector w.r.t. the current position trajectory
     */
    virtual ManifoldState_t::Tangent lift(const ManifoldState_t& m) override
    {
        ManifoldState_t::Tangent t = ManifoldState_t::Tangent::Zero();
        // t.set_pos() = ZERO // important!
        t.set_vel(m.vel().log());
        return t;
    }

protected:
    double dt_;
};


int main(int argc, char** argv)
{
    std::cout << std::fixed;

    const bool use_single_shooting = true;  // toggle between single and multiple shooting

    const size_t N = 17;
    const double dt = 0.1;

    ManifoldState_t x0;
    x0.setIdentity();
    x0.pos() = manif::SO3<double>(M_PI / 2, M_PI / 2, 0.0);
    ct::core::DiscreteArray<ManifoldState_t> x_traj(N + 1, x0);  // init state trajectory, will be overwritten
    ct::core::DiscreteArray<ManifoldState_t::Tangent> b(
        N + 1, ManifoldState_t::Tangent::Zero());                             // defect traj, will be overwritten
    ct::core::DiscreteArray<ct::core::ControlVector<control_dim>> u_traj(N);  // init control traj
    for (size_t i = 0; i < N; i++)
    {
        u_traj[i] = ct::core::ControlVector<control_dim>::Random();
    }

    // choose a random initial state
    // TODO: numerical trouble for more aggressive distributions, since the approximation of the value function becomes really bad?
    for (size_t i = 1; i < N + 1; i++)
    {
        x_traj[i] = ManifoldState_t::Random();
    }

    // create instances of HPIPM and an unconstrained Gauss-Newton Riccati solver
    std::shared_ptr<LQOCSolver<ManifoldState_t, control_dim>> solver(new GNRiccatiSolver<ManifoldState_t, control_dim>);

    // create linear-quadratic optimal control problem containers
    std::shared_ptr<LQOCProblem<ManifoldState_t, control_dim>> problem(
        new LQOCProblem<ManifoldState_t, control_dim>(N));

    // create a discrete-time manifold system
    std::shared_ptr<ct::core::ControlledSystem<ManifoldState_t, control_dim, DISCRETE_TIME>> exampleSystem(
        new DiscrSO3LTITestSystem(dt));
    std::shared_ptr<ct::core::SystemLinearizer<ManifoldState_t, control_dim, DISCRETE_TIME>> linearizer(
        new ct::core::SystemLinearizer<ManifoldState_t, control_dim, DISCRETE_TIME>(exampleSystem));


    // create a cost function
    Eigen::Matrix<double, state_dim, state_dim> Q, Q_final;
    Eigen::Matrix<double, control_dim, control_dim> R;
    Q_final.setZero();
    Q_final.diagonal() << 10000, 10000, 10000, 10000, 100000, 10000;
    Q.setZero();
    Q.diagonal() << 3, 3, 3, 3, 3, 3;
    R.setZero();
    R.diagonal() << 1, 1, 1;
    ManifoldState_t x_final;
    x_final.pos() = manif::SO3<double>(0, 0, 0);
    std::cout << "desired final state: " << x_final << std::endl;
    ManifoldState_t x_nominal = x_final;
    ct::core::ControlVector<control_dim> u_nom = ct::core::ControlVector<control_dim>::Zero();

    // TODO: this currently only works with CostFunctionQuadratic simple (others not ported yet)
    std::shared_ptr<CostFunctionQuadratic<ManifoldState_t, control_dim>> costFunction(
        new CostFunctionQuadraticSimple<ManifoldState_t, control_dim>(Q, R, x_nominal, u_nom, x_final, Q_final));


    // integrate an initial state with the open-loop system to get initial trajectories
    ManifoldState_t x_curr;
    ManifoldState_t::Tangent dx;
    x_curr = x0;
    x_traj.front() = x0;
    std::cout << "integrate an random initial state with the unstable system" << std::endl;
    //std::cout << std::setprecision(4) << "m: " << x_curr << "\t tan: " << x_curr.log() << std::endl;
    for (size_t i = 0; i < N; i++)
    {
        exampleSystem->computeControlledDynamics(x_traj[i], 0, u_traj[i], dx);
        //std::cout << "dx" << dx << std::endl;
        x_curr = x_traj[i].rplus(dx);
        if (use_single_shooting)
            x_traj[i + 1] = x_curr;
        b[i] = dx - x_traj[i + 1].rminus(x_traj[i]);
        //std::cout << "b: " << b[i] << std::endl;
        //std::cout << std::setprecision(10) << "m: " << x_curr << "\t tan: " << x_curr.log() << std::endl;
    }

    size_t nIter = 25;
    for (size_t iter = 0; iter < nIter; iter++)
    {
        // initialize the LQ optimal control problem
        problem->setFromTimeInvariantLinearQuadraticProblem(x_traj, u_traj, *linearizer, *costFunction, b, dt);

        // dynamics transportation for backwards riccati pass and forward solution candidate update
        for (size_t i = 0; i < N; i++)
        {
            auto l = x_traj[i + 1].rminus(x_traj[i]);
            auto l_adj = (l.exp()).adj();
            problem->Adj_x_[i + 1] = l_adj;  // parallel transport matrix / adjoint from stage k+1 to stage k

            // make the necessary overloads, which allows the standard riccati solver to backpropagate on manifolds
            problem->A_[i] = l_adj.transpose() * problem->A_[i];
            problem->B_[i] = l_adj.transpose() * problem->B_[i];
            problem->b_[i] = l_adj.transpose() * problem->b_[i];
        }

        // set the problem pointers
        solver->setProblem(problem);

        // allocate memory (if required)
        solver->initializeAndAllocate();

        // solve the problem
        solver->solve();

        // postprocess data
        solver->compute_lv();
        solver->computeFeedbackMatrices();
        solver->computeStatesAndControls();

        // retrieve solutions from solver
        auto xSol = solver->getSolutionState();
        auto uSol = solver->getSolutionControl();
        ct::core::FeedbackArray<state_dim, control_dim> KSol = solver->getSolutionFeedback();
        ct::core::ControlVectorArray<control_dim> lv_sol = solver->get_lv();

        x_curr = x0;
        ct::core::DiscreteArray<ManifoldState_t> x_traj_prev = x_traj;
        double d_cum_sum = 0;
        double dx_cum_sum = 0;
        double cost_sum = 0;
        for (size_t i = 0; i < N; i++)
        {
            dx.setZero();

            if (use_single_shooting)
            {
                ManifoldState_t::Tangent x_err = x_traj[i].rminus(x_traj_prev[i]);
                u_traj[i] += lv_sol[i] + KSol[i] * x_err;
                exampleSystem->computeControlledDynamics(x_traj[i], i * dt, u_traj[i], dx);
                x_traj[i + 1] = x_traj[i] + dx;
            }
            else  // multiple shooting
            {
                u_traj[i] += uSol[i];
                x_traj[i + 1] = x_traj[i + 1].rplus(xSol[i + 1]);
                exampleSystem->computeControlledDynamics(x_traj[i], i * dt, u_traj[i], dx);
            }

            // compute defect
            b[i] = dx - (x_traj[i + 1].rminus(x_traj[i]));

            // compute update norms
            d_cum_sum += b[i].coeffs().norm();
            dx_cum_sum += xSol[i + 1].coeffs().norm();

            // compute intermediate cost
            costFunction->setCurrentStateAndControl(x_traj[i], u_traj[i], i * dt);
            cost_sum += costFunction->evaluateIntermediate();
        }

        // compute terminal cost
        costFunction->setCurrentStateAndControl(x_traj.back(), u_traj.back(), N * dt);
        cost_sum += costFunction->evaluateTerminal();

        std::cout << std::setprecision(10) << "d_norm: \t " << d_cum_sum << "\t dx_norm: \t" << dx_cum_sum
                  << " \t Jcost: " << cost_sum << std::endl;
    }  // end iter
    return 1;
}