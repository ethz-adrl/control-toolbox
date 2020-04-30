
#include <ct/optcon/optcon.h>
#include <ct/optcon/solver/lqp/AugGNRiccatiSolver.hpp>
#include <ct/optcon/solver/lqp/AugGNRiccatiSolver-impl.hpp>


using namespace ct::core;
using namespace ct::optcon;
const bool verbose = true;


template <typename POS_MAN, typename VEL_MAN, typename POS_TANGENT, typename VEL_TANGENT>
class CompositeManifold;  // forward declaratoin

template <typename POS_MAN, typename VEL_MAN, typename POS_TANGENT, typename VEL_TANGENT>
class CompositeManifoldTangent
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr size_t DoF = POS_TANGENT::DoF + VEL_TANGENT::DoF;
    using Scalar = double;
    using Jacobian = Eigen::Matrix<Scalar, DoF, DoF>;
    using OptJacobianRef = tl::optional<Eigen::Ref<Jacobian>>;

    CompositeManifoldTangent() : t_pos_(POS_TANGENT::Zero()), t_vel_(VEL_TANGENT::Zero()) { coeffs_.setZero(); }
    CompositeManifoldTangent(const POS_TANGENT& p, const VEL_TANGENT& v) : t_pos_(p), t_vel_(v)
    {
        coeffs_ << t_pos_.coeffs(), t_vel_.coeffs();
    }

    template <typename _EigenDerived>
    CompositeManifoldTangent(const Eigen::MatrixBase<_EigenDerived>& v)
        : t_pos_(v.template head<POS_TANGENT::DoF>()), t_vel_(v.template tail<VEL_TANGENT::DoF>()), coeffs_(v)
    {
        // TODO a dimension assert would not hurt
    }

    CompositeManifoldTangent(const CompositeManifoldTangent& resulting_t)
        : t_pos_(resulting_t.t_pos_), t_vel_(resulting_t.t_vel_), coeffs_(resulting_t.coeffs_)
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
        coeffs_ << t_pos_.coeffs(), t_vel_.coeffs();
    }
    void setZero()
    {
        t_pos_.setZero();
        t_vel_.setZero();
        coeffs_.setZero();
    }
    const Eigen::Matrix<Scalar, DoF, 1>& coeffs() const { return coeffs_; }
    void set_coeffs(const Eigen::Matrix<Scalar, DoF, 1>& c)
    {
        coeffs_ = c;
        t_pos_.coeffs() = c.template head<POS_TANGENT::DoF>();
        t_vel_.coeffs() = c.template tail<VEL_TANGENT::DoF>();
    }

    void set_coeff(const int i, const Scalar val)
    {
        coeffs_(i) = val;
        t_pos_.coeffs() = coeffs_.template head<POS_TANGENT::DoF>();
        t_vel_.coeffs() = coeffs_.template tail<VEL_TANGENT::DoF>();
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
        //std::cout << "now calling exp..." << std::endl;
        if (J_m_t)
        {
            throw std::runtime_error("J-m-resulting_t in tangent exp not impl yet.");
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
        coeffs_.template head<POS_TANGENT::DoF>() = p.coeffs();
    }
    void set_vel(const VEL_TANGENT& v)
    {
        t_vel_ = v;
        coeffs_.template tail<VEL_TANGENT::DoF>() = v.coeffs();
    }
    const VEL_TANGENT& vel() const { return t_vel_; }
    //Scalar& operator()(int idx) { return coeffs()(idx); }
    const Scalar& operator()(int idx) const { return coeffs()(idx); }
    //auto noalias() { return CompositeManifoldTangent<POS_MAN, VEL_MAN, POS_TANGENT, VEL_TANGENT>(*this); /* coeffs().noalias();*/ }  // TODO: sort this out
protected:
    POS_TANGENT t_pos_;
    VEL_TANGENT t_vel_;
    Eigen::Matrix<Scalar, DoF, 1> coeffs_;
};

template <typename _Stream, typename t1, typename t2, typename t3, typename t4>
_Stream& operator<<(_Stream& s, const CompositeManifoldTangent<t1, t2, t3, t4>& resulting_t)
{
    s << resulting_t.coeffs().transpose();
    return s;
}

template <class _EigenDerived, typename t1, typename t2, typename t3, typename t4>
auto operator*(const Eigen::MatrixBase<_EigenDerived>& J, const CompositeManifoldTangent<t1, t2, t3, t4>& resulting_t)
{
    return J * resulting_t.coeffs();
}

template <typename t1, typename t2, typename t3, typename t4>
auto operator*(const CompositeManifoldTangent<t1, t2, t3, t4>& resulting_t, const double& s)
{
    return resulting_t.coeffs() * s;
}

template <class _EigenDerived, typename t1, typename t2, typename t3, typename t4>
auto operator+(const Eigen::MatrixBase<_EigenDerived>& tb, const CompositeManifoldTangent<t1, t2, t3, t4>& resulting_t)
{
    CompositeManifoldTangent<t1, t2, t3, t4> res;
    CompositeManifoldTangent<t1, t2, t3, t4> rhs(tb);
    res.set_pos(resulting_t.pos() + rhs.pos());
    res.set_vel(resulting_t.vel() + rhs.vel());
    return res;
}


template <typename POS_MAN, typename VEL_MAN, typename POS_TANGENT, typename VEL_TANGENT>
class CompositeManifold
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Tangent = CompositeManifoldTangent<POS_MAN, VEL_MAN, POS_TANGENT, VEL_TANGENT>;
    static constexpr size_t TangentDim = Tangent::DoF;
    static constexpr size_t Dim = POS_MAN::Dim + VEL_MAN::Dim + 1;  // todo: this is hacky

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

    const Eigen::Matrix<Scalar, Dim, 1> coeffs() const  // todo: return by value is bad
    {
        Eigen::Matrix<Scalar, Dim, 1> c;
        c << m_pos_.coeffs(), m_vel_.coeffs();
        return c;
    }

    CompositeManifold operator+(const Tangent& resulting_t) const { return rplus(resulting_t); }
    //CompositeManifold operator+=(const Tangent& resulting_t)
    //{
    //    *this = this->rplus(resulting_t);
    //    return *this;  // todo test
    //}

    /**
     * @brief right plus for composite manifold
     * NOTE: the operator also has to parallel transport velocities to the new tangent plane.
     * Since for rplus, the rhs is expressed in the tangent space of the lhs, we choose: "first add, then transport"
     */
    CompositeManifold rplus(const Tangent& resulting_t, OptJacobianRef Jl = {}, OptJacobianRef Jr = {}) const
    {
        typename POS_MAN::Jacobian pJl, pJr;
        typename VEL_MAN::Jacobian vJl, vJr;

        VEL_MAN v_new_local = vel().rplus(resulting_t.vel(), vJl, vJr);

        VEL_MAN v_new_transported(resulting_t.pos().exp().adj().transpose() * v_new_local.coeffs());

        CompositeManifold c;
        c.vel() = v_new_transported;
        //c.vel() = c.vel().rplus(resulting_t.vel(), vJl, vJr);
        c.pos() = pos().rplus(resulting_t.pos(), pJl, pJr);  // todo make jacobians optional

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

        Tangent resulting_t;
        resulting_t.set_pos(pos().rminus(rhs.pos(), pJl, pJr));  // todo make jacobians optional
        VEL_MAN v_lhs_transported(resulting_t.pos().exp().adj() * vel().coeffs());

        VEL_TANGENT v_new_local = v_lhs_transported.rminus(rhs.vel(), vJl, vJr);
        resulting_t.set_vel(v_new_local);

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

        return resulting_t;
    }


    Tangent log(OptJacobianRef J_t_m = {}) const
    {
        if (J_t_m)
            throw std::runtime_error("J_t_m compuation not defined.");

        return rminus(CompositeManifold::Identity(), J_t_m);
        //resulting_t.set_pos(m_pos_.log());
        //resulting_t.set_vel(m_vel_.log());
        //return resulting_t;
    }

    /**
     * @brief The question
     * The fundamental question remains: what is the velocity adjoint?
     *  - multiple shooting: in the forward update rule, the computed velocities want to be transported to the next states manifold coordinate system
     *  - single shooting: seems to be much more stable when velocity part adjoint is identity -- is the value function more stable???
     */
    Jacobian adj() const
    {
        Jacobian J = Jacobian::Zero();
        J.template topLeftCorner<POS_MAN::DoF, POS_MAN::DoF>() = m_pos_.adj();
        J.template bottomRightCorner<VEL_MAN::DoF, VEL_MAN::DoF>() =
            m_pos_.adj();  //.setIdentity();  // = m_pos_.adj(); // TODO: what exactly is it?
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
     * @brief the log operator is defined as expressing the tangent vector w.r.resulting_t. m_ref_
     */
    virtual ManifoldState_t::Tangent lift(const ManifoldState_t& m) override
    {
        ManifoldState_t::Tangent resulting_t = ManifoldState_t::Tangent::Zero();
        //resulting_t.set_pos( "zero...");
        resulting_t.set_vel(m.vel().log());
        return resulting_t;
    }

    //virtual ManifoldState_t retract(const ManifoldState_t::Tangent& resulting_t) override {
    //     throw std::runtime_error(" retract not implemetned.");
    //     /*return m_ref_.rplus(resulting_t);*/ }
protected:
    //ManifoldState_t m_ref_;
    double dt_;
};


// TODO: make this a unit test
//void testParallelTransport()
//{
//    auto origin = manif::SE2<double>(0, 0, 0);
//    Eigen::Matrix3d b;
//    b.setIdentity();  // some basis, assumed to be expressed in the manifolds over which we iterate
//
//    for (double theta = M_PI / 2; theta <= 2 * M_PI; theta += M_PI / 2)
//    {
//        std::cout << "case theta = " << theta << std::endl;
//        auto m = manif::SE2<double>(0, 0, theta);
//        Eigen::Matrix3d Jl, Jr;  // jacobians
//        auto d_rminus = m.rminus(origin, Jl, Jr);
//        auto adj_rminus = d_rminus.exp().adj();  // the adjoint for transportation to the origin
//
//        auto d_between = m.between(origin);
//        auto adj_between = d_rminus.exp().adj();
//
//        // ASSERT_TRUE(adj_rminus.isApprox(adj_between));
//
//        std::cout << adj_rminus * b << std::endl << std::endl;   // express basis b in origin-manifold
//        std::cout << adj_between * b << std::endl << std::endl;  // express basis b in origin-manifold
//        // std::cout << Jr * Jl * b << std::endl << std::endl;  // TODO: how does the Jac relate to the adjoint here?
//        std::cout << std::endl;
//    }
//
//    // auto m_pi_2 = manif::SE2<double>(0, 0, M_PI / 2);
//    // auto m_pi = manif::SE2<double>(0, 0, M_PI);
//    // auto m_mpi_2 = manif::SE2<double>(0, 0, -M_PI / 2);
//    //
//    //
//    // std::cout << m_0.adj() * b << std::endl << std::endl;
//    // std::cout << m_pi_2.adj() * b << std::endl << std::endl;
//    // std::cout << m_pi.adj() * b << std::endl << std::endl;
//    // std::cout << m_mpi_2.adj() * b << std::endl << std::endl;
//}

int main(int argc, char** argv)
{
    //ManifoldState_t x_test;
    //x_test.setIdentity();
    //Eigen::Vector3d vel_init(1.0, 0.0, 0.0);
    //x_test.vel() = manif::R3d(vel_init);
    ////x0.pos() = manif::SO3<double>(0, 0, M_PI);
    //std::cout << "m: " << x_test << std::endl;
    //ManifoldState_t::Tangent t_test;
    //t_test.set_coeff(2, M_PI / 2);
    //t_test.set_coeff(3, 1);
    //
    //std::cout << "resulting_t: " << t_test << std::endl;
    //ManifoldState_t newX = x_test.rplus(t_test);
    //std::cout << "newX: " << newX << std::endl;
    //auto t_reverse = newX.rminus(x_test);
    //std::cout << "t_reverse:" << t_reverse << std::endl;
    //std::cout << "t_reverse.exp() " << t_reverse.exp() << std::endl;
    //std::cout << "t_reverse.exp().log() " << t_reverse.exp().log() << std::endl;
    //
    //std::cout << "parallel transport study: " << std::endl;
    //auto ADJ = t_reverse.exp().adj();
    //std::cout << "adj: " << ADJ << std::endl;
    //std::cout << "adj*t_test: " << ADJ * t_test << std::endl;
    //exit(0);

    std::cout << std::fixed;

    const bool use_single_shooting = true;  // toggle between single and multiple shooting

    const size_t N = 17;
    const double dt = 0.1;

    ManifoldState_t x0;
    x0.setIdentity();
    x0.pos() = manif::SO3<double>(M_PI/2, M_PI/2, 0.0);
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
    std::shared_ptr<LQOCSolver<ManifoldState_t, control_dim>> gnRiccatiSolver(
        new GNRiccatiSolver<ManifoldState_t, control_dim>);
    std::shared_ptr<LQOCSolver<ManifoldState_t, control_dim>> augGnRiccatiSolver(
        new AugGNRiccatiSolver<ManifoldState_t, control_dim>);

    // store them, and identifying names, in a vectors
    std::vector<std::shared_ptr<LQOCSolver<ManifoldState_t, control_dim>>> lqocSolvers;
    lqocSolvers.push_back(gnRiccatiSolver);
    lqocSolvers.push_back(augGnRiccatiSolver);
    std::vector<std::string> solverNames = {"Riccati", "AugRiccati"};

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
        new DiscrSO3LTITestSystem(dt));
    std::shared_ptr<ct::core::SystemLinearizer<ManifoldState_t, control_dim, DISCRETE_TIME>> linearizer(
        new ct::core::SystemLinearizer<ManifoldState_t, control_dim, DISCRETE_TIME>(exampleSystem));


    // create a cost function
    Eigen::Matrix<double, state_dim, state_dim> Q, Q_final;
    Eigen::Matrix<double, control_dim, control_dim> R;
    Q_final.setZero();
    Q_final.diagonal() << 10000, 10000, 10000, 10000, 100000, 10000;
    Q.setZero();
    // Q.diagonal() << 3, 3, 3, 3, 3, 3;
    R.setZero();
    R.diagonal() << 1, 1, 1;
    ManifoldState_t x_final;
    x_final.pos() = manif::SO3<double>(0, 0, 0);
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
    std::cout << std::setprecision(4) << "m: " << x_curr << "\resulting_t tan: " << x_curr.log() << std::endl;
    for (size_t i = 0; i < N; i++)
    {
        exampleSystem->computeControlledDynamics(x_traj[i], 0, u_traj[i], dx);
        std::cout << "dx" << dx << std::endl;
        x_curr = x_traj[i].rplus(dx);
        if (use_single_shooting)
            x_traj[i + 1] = x_curr;
        b[i] = dx - x_traj[i + 1].rminus(x_traj[i]);
        std::cout << "b: " << b[i] << std::endl;
        std::cout << std::setprecision(10) << "m: " << x_curr << "\resulting_t tan: " << x_curr.log() << std::endl;
    }

    size_t nIter = 25;
    for (size_t iter = 0; iter < nIter; iter++)
    {
        // initialize the optimal control problems for both solvers
        problems[0]->setFromTimeInvariantLinearQuadraticProblem(x_traj, u_traj, *linearizer, *costFunction, b, dt);
        problems[1]->setFromTimeInvariantLinearQuadraticProblem(x_traj, u_traj, *linearizer, *costFunction, b, dt);


        // HACKY corrections // TODO: move somewhere meaningful
        for (size_t idx : {0, 1})
        {
            // intermediate stages cost transportation
            for (size_t i = 0; i < N; i++)
            {
                auto e = x_nominal.rminus(x_traj[i]);
                // compute PT matrix w.r.resulting_t. current ref traj // TODO: clarify formulation of error in cost function
                auto e_adj = (e.exp()).adj();
                problems[idx]->Q_[i] = e_adj * problems[idx]->Q_[i] *
                                       e_adj.transpose();  // TODO: sort out that thing with the cost function
                problems[idx]->qv_[i] = e_adj * problems[idx]->qv_[i];
            }
            // terminal stage transportation
            // TODO: clarifiy formulation of error in cost function
            // auto e = x_final.rminus(x_traj[N]);  // compute PT matrix w.r.resulting_t. current ref traj
            // auto e_adj = (e.exp()).adj();
            //std::cout << "cost adj" << std::endl << e_adj << std::endl;
            //problems[idx]->Q_.back() = e_adj * problems[idx]->Q_.back() * e_adj.transpose();
            //problems[idx]->qv_.back() = e_adj * problems[idx]->qv_.back();
            //std::cout << "problems[idx]->Q_.back()" << std::endl << problems[idx]->Q_.back() << std::endl;
            //std::cout << "problems[idx]->qv_.back()" << std::endl << problems[idx]->qv_.back() << std::endl;

            // dynamics transportation
            for (size_t i = 0; i < N; i++)
            {
                // std::cout << "dyn transport matrices" << std::endl;
                //Eigen::Matrix3d Jl, Jr;
                auto l = x_traj[i + 1].rminus(x_traj[i]);
                auto l_adj = (l.exp()).adj();
                problems[idx]->Adj_x_[i + 1] = l_adj;  // parallel transport matrix / adjoint from stage k+1 to stage k

                if (idx == 0)  // make the corrections for the standard riccati solver
                {
                    problems[idx]->A_[i] = l_adj.transpose() * problems[idx]->A_[i];
                    problems[idx]->b_[i] = l_adj.transpose() * problems[idx]->b_[i];
                    problems[idx]->B_[i] = l_adj.transpose() * problems[idx]->B_[i];
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

        auto xSol_aug_riccati = lqocSolvers[1]->getSolutionState();
        auto uSol_aug_riccati = lqocSolvers[1]->getSolutionControl();
        ct::core::FeedbackArray<state_dim, control_dim> KSol_aug_riccati = lqocSolvers[1]->getSolutionFeedback();
        ct::core::ControlVectorArray<control_dim> lv_sol_aug_riccati = lqocSolvers[1]->get_lv();

        // compare the quantities
        //for (size_t i = 0; i < lv_sol_riccati.size(); i++)
        //{
        //    if ((lv_sol_riccati[i] - lv_sol_aug_riccati[i]).array().abs().maxCoeff() > 1e-8)
        //    {
        //        std::cout << std::setprecision(10) << lv_sol_riccati[i].transpose() << std::endl;
        //        std::cout << std::setprecision(10) << lv_sol_aug_riccati[i].transpose() << std::endl;
        //        throw std::runtime_error("lv solutions do not match");
        //    }
        //}
        //for (size_t i = 0; i < KSol_riccati.size(); i++)
        //{
        //    if (KSol_riccati[i].isApprox(KSol_aug_riccati[i], 1e-6) == false)
        //        throw std::runtime_error("K solutions do not match");
        //}
        //
        //for (size_t i = 0; i < uSol_riccati.size(); i++)
        //{
        //    if ((uSol_riccati[i] - uSol_aug_riccati[i]).array().abs().maxCoeff() > 1e-8)
        //    {
        //        std::cout << "for index " << i << std::endl;
        //        std::cout << std::setprecision(10) << uSol_riccati[i].transpose() << std::endl;
        //        std::cout << std::setprecision(10) << uSol_aug_riccati[i].transpose() << std::endl;
        //        throw std::runtime_error("u solutions do not match");
        //    }
        //}
        //for (size_t i = 0; i < xSol_riccati.size(); i++)
        //{
        //    if ((xSol_riccati[i] - xSol_aug_riccati[i]).coeffs().array().abs().maxCoeff() > 1e-8)
        //    {
        //        std::cout << xSol_riccati[i].transpose() << std::endl;
        //        std::cout << xSol_aug_riccati[i].transpose() << std::endl;
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
        //    std::cout << "m:" << x_solver_direct[j] << "\resulting_t dx:" << xSol_riccati[j].transpose()
        //              << "\resulting_t -- rot diff norm(): " << angularDiff << std::endl;
        //}

        std::cout << std::setprecision(4) << "Forward integrated closed-loop solution, iter :" << iter << std::endl;
        x_curr = x0;
        std::cout << "m: " << x_curr << "\t tan: " << x_curr.log() << std::endl;
        ct::core::DiscreteArray<ManifoldState_t> x_traj_prev = x_traj;
        double d_cum_sum = 0;
        double dx_cum_sum = 0;
        double cost_sum = 0;
        for (size_t i = 0; i < N; i++)
        {
            dx.setZero();
            //Eigen::Quaterniond old_rot(x_traj[i].w(), x_traj[i].x(), x_traj[i].y(), x_traj[i].z());

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
                x_traj[i + 1] = x_traj[i + 1].rplus(xSol_riccati[i + 1]);
                exampleSystem->computeControlledDynamics(x_traj[i], i * dt, u_traj[i], dx);
            }

            //Eigen::Quaterniond new_rot(x_traj[i + 1].w(), x_traj[i + 1].x(), x_traj[i + 1].y(), x_traj[i + 1].z());
            //std::cout << "m: " << x_traj[i + 1] << "\resulting_t log: " << x_traj[i + 1].log() << " u: " << u_traj[i].transpose()
            //          << std::endl;

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
        std::cout << std::endl << std::endl;
    }  // end iter
    return 1;
}