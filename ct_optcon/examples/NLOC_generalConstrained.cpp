/*!
 * \example NLOC_generalConstrained.cpp
 *
 * This example shows how to use general constraints alongside NLOC and requires HPIPM to be installed
 * The unconstrained Riccati backward-pass is replaced by a high-performance interior-point
 * constrained linear-quadratic Optimal Control solver (HPIPM).
 *
 */

#include <ct/optcon/optcon.h>
#include "exampleDir.h"
#include "plotResultsPointMass.h"

using namespace ct::core;
using namespace ct::optcon;


/* state and control input dimension of the point mass. States: x, y, v_x, v_y.
 * Controls: F_x, F_y */
static const size_t STATE_DIM = 4;
static const size_t CONTROL_DIM = 2;

/*!
 * @brief Dynamics class for a second order system, represented by a moving point mass
 * under force actuation
 */

// class for a point mass (second order system) in two-dimensional space
class PointMass : public ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM>
{
public:
    typedef ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM> Base;
    typedef typename Base::time_t time_t;

    PointMass(const double m, std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM>> controller = nullptr)
        : ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM>(controller, ct::core::SYSTEM_TYPE::SECOND_ORDER), m_(m)
    {
    }

    PointMass(const PointMass& other) : ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM>(other), m_(other.m_) {}
    // deep clone
    PointMass* clone() const override { return new PointMass(*this); }
    virtual ~PointMass() = default;

    // compute the system dynamics
    virtual void computeControlledDynamics(const ct::core::StateVector<STATE_DIM>& state,
        const ct::core::Time& t,
        const ct::core::ControlVector<CONTROL_DIM>& control,
        ct::core::StateVector<STATE_DIM>& dxdt) override
    {
        dxdt(0) = state(2);
        dxdt(1) = state(3);
        dxdt(2) = control(0) / m_;
        dxdt(3) = control(1) / m_;
    }

private:
    double m_;  // point mass
};

/*!
 * @brief A simple 1d constraint term.
 *
 * This term implements the general inequality constraints
 * \f$ (x - x0)^2 + (y - y0)^2 \geq r^2 \f$
 * where \f$ r \f$ denotes the radius and \f$ (x0, y0) \f$ the position of an obstacle.
 *
 * This constraint encodes a behavior where a moving object avoids obstacles which
 * are represented by a circle.
 */

// class representing a general constraint that ensures obstacle avoidance
class ObstacleConstraintSimple : public ct::optcon::ConstraintBase<STATE_DIM, CONTROL_DIM>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef ct::optcon::ConstraintBase<STATE_DIM, CONTROL_DIM> Base;
    typedef ct::core::StateVector<STATE_DIM> state_vector_t;
    typedef ct::core::ControlVector<CONTROL_DIM> control_vector_t;
    typedef Eigen::Matrix<double, 2, 1> Vector2s;

    ObstacleConstraintSimple(const Vector2s x0, const double r) : x0_(x0), r_(r)
    {
        Base::lb_.resize(1);
        Base::ub_.resize(1);
        Base::lb_.setConstant(0.0);
        Base::ub_.setConstant(std::numeric_limits<double>::max());
    }

    ObstacleConstraintSimple(const ObstacleConstraintSimple& other)
    {
        Base::lb_.resize(1);
        Base::ub_.resize(1);
        Base::lb_.setConstant(0.0);
        Base::ub_.setConstant(std::numeric_limits<double>::max());
    }

    virtual ObstacleConstraintSimple* clone() const override { return new ObstacleConstraintSimple(*this); }
    virtual size_t getConstraintSize() const override { return 1; }
    virtual Eigen::VectorXd evaluate(const state_vector_t& x, const control_vector_t& u, const double t) override
    {
        Eigen::Matrix<double, 1, 1> val;
        // returns a number > 0 if point x is outside of the circle
        val.template segment<1>(0) << pow((x(0) - x0_(0)), 2) / pow(r_, 2) + pow((x(1) - x0_(1)), 2) / pow(r_, 2) -
                                          double(1.0);
        return val;
    }

    // provide the constraint equation for automatic differentiation with CppadCg
    virtual Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> evaluateCppadCg(
        const ct::core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
        const ct::core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override
    {
        Eigen::Matrix<ct::core::ADCGScalar, 1, 1> val;
        val.template segment<1>(0) << (x(0) - x0_(0)) * (x(0) - x0_(0)) + (x(1) - x0_(1)) * (x(1) - x0_(1)) -
                                          ct::core::ADCGScalar(r_) * ct::core::ADCGScalar(r_);
        return val;
    }

private:
    Vector2s x0_;  // center of the circular obstacle
    double r_;     // radius of the circular obstacle
};

int main(int argc, char** argv)
{
    /* STEP 1: set up the Nonlinear Optimal Control Problem*/
    /* STEP 1-A: create a instance of the differential drive dynamics for the
     * optimal control problem.*/
    const double m = 1;

    std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM>> pointMass(new PointMass(m));

    /* STEP 1-B: System Linearizer */
    std::shared_ptr<ct::core::SystemLinearizer<STATE_DIM, CONTROL_DIM>> adLinearizer(
        new ct::core::SystemLinearizer<STATE_DIM, CONTROL_DIM>(pointMass));

    /* STEP 1-C: create a cost function. */
    std::shared_ptr<ct::optcon::TermQuadratic<STATE_DIM, CONTROL_DIM>> intermediateCost(
        new ct::optcon::TermQuadratic<STATE_DIM, CONTROL_DIM>());
    std::shared_ptr<ct::optcon::TermQuadratic<STATE_DIM, CONTROL_DIM>> finalCost(
        new ct::optcon::TermQuadratic<STATE_DIM, CONTROL_DIM>());
    bool verbose = true;
    intermediateCost->loadConfigFile(
        ct::optcon::exampleDir + "/nlocCost_ObstacleConstraint.info", "intermediateCost", verbose);
    finalCost->loadConfigFile(ct::optcon::exampleDir + "/nlocCost_ObstacleConstraint.info", "finalCost", verbose);

    // Container for analytical cost function
    std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>> costFunction(
        new ct::optcon::CostFunctionAnalytical<STATE_DIM, CONTROL_DIM>());
    costFunction->addIntermediateTerm(intermediateCost);
    costFunction->addFinalTerm(finalCost);

    /* STEP 1-D: set up the general constraints */
    Eigen::Matrix<double, 2, 1> x_c;
    x_c << 7.0, 5.0;  // the center position of the obstacle

    const double r = 3.0;  // the radius of the obstacle
    std::shared_ptr<ObstacleConstraintSimple> obstacleConstraint(new ObstacleConstraintSimple(x_c, r));
    obstacleConstraint->setName("ObstacleConstraint");

    //constraint container for general constraints with automatic differentiation (AD)
    std::shared_ptr<ct::optcon::ConstraintContainerAD<STATE_DIM, CONTROL_DIM>> generalConstraints(
        new ct::optcon::ConstraintContainerAD<STATE_DIM, CONTROL_DIM>());

    // add and initialize general constraint terms
    generalConstraints->addIntermediateConstraint(obstacleConstraint, verbose);
    generalConstraints->addTerminalConstraint(obstacleConstraint, verbose);
    generalConstraints->initialize();

    /* STEP 1-E: initialization with initial state and desired time horizon */
    ct::core::StateVector<STATE_DIM> x0;
    x0.setZero();

    ct::core::Time timeHorizon = 5.0;  // and a final time horizon in [sec]

    // STEP 1-E: create and initialize an "optimal control problem"
    ct::optcon::ContinuousOptConProblem<STATE_DIM, CONTROL_DIM> optConProblem(
        timeHorizon, x0, pointMass, costFunction, adLinearizer);

    // add the general constraints to the optimal control
    // problem
    optConProblem.setGeneralConstraints(generalConstraints);

    /* STEP 2: set up a nonlinear optimal control solver. */

    /* STEP 2-A: Create the settings. */
    ct::optcon::NLOptConSettings nloc_settings;
    nloc_settings.load(ct::optcon::exampleDir + "/nlocSolver_ObstacleConstraint.info", true, "ilqr");

    /* STEP 2-B: provide an initial guess */
    // calculate the number of time steps K
    size_t K = nloc_settings.computeK(timeHorizon);

    /* design trivial initial controller for NLOC. */
    ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM> u0_fb(K, ct::core::FeedbackMatrix<STATE_DIM, CONTROL_DIM>::Zero());
    ct::core::ControlVectorArray<CONTROL_DIM> u0_ff(K, ct::core::ControlVector<CONTROL_DIM>::Zero());
    ct::core::StateVectorArray<STATE_DIM> x_ref_init(K + 1, x0);
    ct::optcon::NLOptConSolver<STATE_DIM, CONTROL_DIM>::Policy_t initController(
        x_ref_init, u0_ff, u0_fb, nloc_settings.dt);

    // STEP 2-C: create an NLOptConSolver instance
    ct::optcon::NLOptConSolver<STATE_DIM, CONTROL_DIM> nloc(optConProblem, nloc_settings);

    // set the initial guess
    nloc.setInitialGuess(initController);

    // STEP 3: solve the optimal control problem
    nloc.solve();

    // STEP 4: retrieve the solution
    ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM> solution = nloc.getSolution();

    // plot the output
    plotResultsPointMass<STATE_DIM, CONTROL_DIM>(
        solution.x_ref(), solution.uff(), solution.time(), r, x_c(0, 0), x_c(1, 0));

    return 0;
}
