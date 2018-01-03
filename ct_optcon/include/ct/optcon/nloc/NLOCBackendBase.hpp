/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <atomic>

#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>
#include <ct/optcon/solver/OptConSolver.h>

#include <ct/optcon/problem/LQOCProblem.hpp>

#include <ct/optcon/solver/lqp/GNRiccatiSolver.hpp>
#include <ct/optcon/solver/lqp/HPIPMInterface.hpp>

#include <ct/optcon/solver/NLOptConSettings.hpp>

#include "NLOCResults.hpp"

#ifdef MATLAB
#include <ct/optcon/matlab.hpp>
#endif

#define SYMPLECTIC_ENABLED        \
    template <size_t V, size_t P, size_t ST> \
    typename std::enable_if<(V > 0 && P > 0 && (V+P==ST)), void>::type
#define SYMPLECTIC_DISABLED       \
    template <size_t V, size_t P, size_t ST> \
    typename std::enable_if<(V <= 0 || P <= 0 || (V+P!=ST)), void>::type

namespace ct {
namespace optcon {


/*!
 * \ingroup GNMS
 *
 * \brief C++ implementation of GNMS.
 *
 *  The implementation and naming is based on the reference below. In general, the code follows this convention:
 *  X  <- Matrix (upper-case in paper)
 *  xv <- vector (lower-case bold in paper)
 *  x  <- scalar (lower-case in paper)
 *
 */

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR = double>
class NLOCBackendBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t state_dim = STATE_DIM;
    static const size_t control_dim = CONTROL_DIM;

    typedef NLOptConSettings Settings_t;
    typedef core::StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR> Policy_t;

    typedef OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR> OptConProblem_t;

    typedef LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR> LQOCProblem_t;
    typedef LQOCSolver<STATE_DIM, CONTROL_DIM, SCALAR> LQOCSolver_t;

    typedef core::Sensitivity<STATE_DIM, CONTROL_DIM, SCALAR> Sensitivity_t;

    typedef ct::core::StateVectorArray<STATE_DIM, SCALAR> StateVectorArray;
    typedef std::shared_ptr<StateVectorArray> StateVectorArrayPtr;

    typedef ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> ControlVectorArray;
    typedef std::shared_ptr<ControlVectorArray> ControlVectorArrayPtr;

    typedef std::vector<StateVectorArrayPtr, Eigen::aligned_allocator<StateVectorArrayPtr>> StateSubsteps;
    typedef std::shared_ptr<StateSubsteps> StateSubstepsPtr;

    typedef std::vector<ControlVectorArrayPtr, Eigen::aligned_allocator<ControlVectorArrayPtr>> ControlSubsteps;
    typedef std::shared_ptr<ControlSubsteps> ControlSubstepsPtr;

    using ControlMatrix = ct::core::ControlMatrix<CONTROL_DIM, SCALAR>;
    using ControlMatrixArray = ct::core::ControlMatrixArray<CONTROL_DIM, SCALAR>;
    using StateMatrixArray = ct::core::StateMatrixArray<STATE_DIM, SCALAR>;
    using StateControlMatrixArray = ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR>;
    using FeedbackArray = ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR>;
    using TimeArray = ct::core::tpl::TimeArray<SCALAR>;

    using state_matrix_t = Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM>;
    using control_matrix_t = Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM>;
    using control_state_matrix_t = Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM>;
    using state_control_matrix_t = Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM>;

    using state_vector_t = core::StateVector<STATE_DIM, SCALAR>;
    using control_vector_t = core::ControlVector<CONTROL_DIM, SCALAR>;
    using feedback_matrix_t = core::FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>;

    using scalar_t = SCALAR;
    using scalar_array_t = std::vector<SCALAR, Eigen::aligned_allocator<SCALAR>>;


    NLOCBackendBase(const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem, const Settings_t& settings);

    NLOCBackendBase(const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem,
        const std::string& settingsFile,
        bool verbose = true,
        const std::string& ns = "alg");

    virtual ~NLOCBackendBase();

    //! configure the solver
    /**
	 * Configure the solver
	 * @param settings solver settings
	 */
    virtual void configure(const Settings_t& settings);

    //! get the current SLQsolver settings
    const Settings_t& getSettings() const;

    /*!
	 * Set the initial guess used by the solver (not all solvers might support initial guesses)
	 */
    void setInitialGuess(const Policy_t& initialGuess);


    /*!
	 * \brief Change the time horizon the solver operates on.
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct time horizon set.
	 */
    void changeTimeHorizon(const SCALAR& tf);

    SCALAR getTimeHorizon();

    int getNumSteps();

    int getNumStepsPerShot();

    SYMPLECTIC_ENABLED initializeSymplecticIntegrators(size_t i);
    SYMPLECTIC_DISABLED initializeSymplecticIntegrators(size_t i);

    SYMPLECTIC_ENABLED integrateSymplectic(size_t threadId,
        ct::core::StateVector<STATE_DIM, SCALAR>& x0,
        const double& t,
        const size_t& steps,
        const double& dt_sim) const;
    SYMPLECTIC_DISABLED integrateSymplectic(size_t threadId,
        ct::core::StateVector<STATE_DIM, SCALAR>& x0,
        const double& t,
        const size_t& steps,
        const double& dt_sim) const;


    /*!
	 * \brief Change the initial state for the optimal control problem
	 */
    void changeInitialState(const core::StateVector<STATE_DIM, SCALAR>& x0);

    /*!
	 * \brief Change the cost function
	 */
    void changeCostFunction(const typename OptConProblem_t::CostFunctionPtr_t& cf);

    /*!
	 * \brief Change the nonlinear system
	 */
    void changeNonlinearSystem(const typename OptConProblem_t::DynamicsPtr_t& dyn);

    /*!
	 * \brief Change the linear system
	 */
    void changeLinearSystem(const typename OptConProblem_t::LinearPtr_t& lin);

    /*!
	 * \brief Change the box constraints
	 */
    void changeBoxConstraints(const typename OptConProblem_t::ConstraintPtr_t& con);

    /*!
	 * \brief Change the general constraints
	 */
    void changeGeneralConstraints(const typename OptConProblem_t::ConstraintPtr_t& con);

    /*!
	 * \brief Direct accessor to the system instances
	 *
	 * \warning{Use this only when performance absolutely matters and if you know what you
	 * are doing. Otherwise use e.g. changeNonlinearSystem() to change the system dynamics
	 * in a safe and easy way. You should especially not change the size of the vector or
	 * modify each entry differently.}
	 * @return
	 */
    std::vector<typename OptConProblem_t::DynamicsPtr_t>& getNonlinearSystemsInstances();

    const std::vector<typename OptConProblem_t::DynamicsPtr_t>& getNonlinearSystemsInstances() const;

    /*!
	 * \brief Direct accessor to the linear system instances
	 *
	 * \warning{Use this only when performance absolutely matters and if you know what you
	 * are doing. Otherwise use e.g. changeLinearSystem() to change the system dynamics
	 * in a safe and easy way. You should especially not change the size of the vector or
	 * modify each entry differently.}
	 * @return
	 */
    std::vector<typename OptConProblem_t::LinearPtr_t>& getLinearSystemsInstances();

    const std::vector<typename OptConProblem_t::LinearPtr_t>& getLinearSystemsInstances() const;

    /*!
	 * \brief Direct accessor to the cost function instances
	 *
	 * \warning{Use this only when performance absolutely matters and if you know what you
	 * are doing. Otherwise use e.g. changeCostFunction() to change the system dynamics
	 * in a safe and easy way. You should especially not change the size of the vector or
	 * modify each entry differently.}
	 * @return
	 */
    std::vector<typename OptConProblem_t::CostFunctionPtr_t>& getCostFunctionInstances();

    const std::vector<typename OptConProblem_t::CostFunctionPtr_t>& getCostFunctionInstances() const;

    /**
	 * @brief      Direct accessor to the box constraint instances
	 *
	 * \warning{Use this only when performance absolutely matters and if you know what you
	 * are doing. Otherwise use e.g. changeCostFunction() to change the system dynamics
	 * in a safe and easy way. You should especially not change the size of the vector or
	 * modify each entry differently.}
	 *
	 * @return     The box constraint instances
	 */
    std::vector<typename OptConProblem_t::ConstraintPtr_t>& getBoxConstraintsInstances();

    const std::vector<typename OptConProblem_t::ConstraintPtr_t>& getBoxConstraintsInstances() const;

    /**
	 * @brief      Direct accessor to the general constraints
	 *
	 * \warning{Use this only when performance absolutely matters and if you know what you
	 * are doing. Otherwise use e.g. changeCostFunction() to change the system dynamics
	 * in a safe and easy way. You should especially not change the size of the vector or
	 * modify each entry differently.}
	 *
	 * @return     The general constraints instances.
	 */
    std::vector<typename OptConProblem_t::ConstraintPtr_t>& getGeneralConstraintsInstances();

    const std::vector<typename OptConProblem_t::ConstraintPtr_t>& getGeneralConstraintsInstances() const;


    /*!
	 * Tests consistency of the instance of the dynamics, linear systems and costs. This is not a test for thread safety.
	 * @return returns true if instances are consistent with each other
	 */
    bool testConsistency();


    //! Export all functions to matlab workspace
    /*!
	  This function can be used for Debugging. It exports all variables to Matlab after each iteration. It also saves
	  the Matlab workspace to a .mat file.
	*/
    void logToMatlab(const size_t& iteration);

    //! log the initial guess to Matlab
    void logInitToMatlab();

    //! return the cost of the solution of the current iteration
    SCALAR getCost() const;

    //! return the sum of the L2-norm of the defects along the solution candidate
    SCALAR getTotalDefect() const;

    void reset();

    const core::StateTrajectory<STATE_DIM, SCALAR> getStateTrajectory() const;

    const core::ControlTrajectory<CONTROL_DIM, SCALAR> getControlTrajectory() const;


    const Policy_t& getSolution();

    const TimeArray& getTimeArray();

    bool isConfigured();

    bool isInitialized();


    //! Retrieve Last Linearized Model
    /*!
	  Retrieve the linearized model computed during the last iteration
	*/
    void retrieveLastLinearizedModel(StateMatrixArray& A, StateControlMatrixArray& B);

    /*!
	 * the prepare Solve LQP Problem method is intended for a special use-case: unconstrained GNMS with pre-solving of the
	 */
    virtual void prepareSolveLQProblem(size_t startIndex);


    virtual void finishSolveLQProblem(size_t endIndex);

    /*!
	 * solve Full LQProblem, e.g. to be used with HPIPM or if we have a constrained problem
	 */
    virtual void solveFullLQProblem();

    //! compute costs of solution candidate
    void updateCosts();

    //! nominal rollout using default thread and member variables for the results. // todo maybe rename (initial rollout?)
    bool nominalRollout();

    //! check problem for consistency
    void checkProblem();

    //! return the current iteration number
    size_t& iteration();

    //! Print iteration summary
    /*!
	 *  This function is automatically called if the printSummary settings is on. It prints out important information
	 *  like cost etc. after each iteration.
	 */
    void printSummary();

    //! perform line-search and update controller for single shooting
    bool lineSearchSingleShooting();

    //! perform line-search and update controller for multiple shooting
    bool lineSearchMultipleShooting();

    //! build LQ approximation around trajectory (linearize dynamics and general constraints, quadratize cost)
    virtual void computeLQApproximation(size_t firstIndex, size_t lastIndex) = 0;

    //! sets the box constraints for the entire time horizon including terminal stage
    void setBoxConstraintsForLQOCProblem();

    //! obtain state update from lqoc solver
    void getStateUpdates();

    //! obtain control update from lqoc solver
    void getControlUpdates();

    //! obtain feedback update from lqoc solver, if provided
    void getFeedback();

    //! reset all defects to zero
    void resetDefects();

    //! update the nominal defects
    void computeDefectsNorm();

    //! integrates the specified shots and computes the corresponding defects
    virtual void rolloutShots(size_t firstIndex, size_t lastIndex) = 0;

    //! do a single threaded rollout and defect computation of the shots - useful for line-search
    bool rolloutShotsSingleThreaded(size_t threadId,
        size_t firstIndex,
        size_t lastIndex,
        ControlVectorArray& u_ff_local,
        StateVectorArray& x_local,
        const StateVectorArray& x_ref_lqr,
        StateVectorArray& xShot,
        StateVectorArray& d,
        StateSubsteps& substepsX,
        ControlSubsteps& substepsU) const;

    //! performLineSearch: execute the line search, possibly with different threading schemes
    virtual SCALAR performLineSearch() = 0;

    //! simple full-step update for state and feedforward control (used for MPC-mode!)
    void doFullStepUpdate();

    void logSummaryToMatlab(const std::string& fileName);

    const SummaryAllIterations<SCALAR>& getSummary() const;

protected:
    //! integrate the individual shots
    bool rolloutSingleShot(const size_t threadId,
        const size_t k,
        ControlVectorArray& u_ff_local,
        StateVectorArray& x_local,
        const StateVectorArray& x_ref_lqr,
        StateVectorArray& xShot,
        StateSubsteps& substepsX,
        ControlSubsteps& substepsU,
        std::atomic_bool* terminationFlag = nullptr) const;

    /*
	bool simpleRollout(
			const size_t threadId,
			const ControlVectorArray& uff,
			const StateVectorArray& x_ref_lqr,
			StateVectorArray& x_local,
			ControlVectorArray& u_recorded
			)const;
			*/

    //! computes the defect between shot and trajectory
    /*!
	 * @param k			index of the shot under consideration
	 * @param x_local	the state trajectory
	 * @param xShot		the shot trajectory
	 * @param d			the defect trajectory
	 */
    void computeSingleDefect(size_t k,
        const StateVectorArray& x_local,
        const StateVectorArray& xShot,
        StateVectorArray& d) const;


    //! Computes the linearized Dynamics at a specific point of the trajectory
    /*!
	  This function calculates the linearization, i.e. matrices A and B in \f$ \dot{x} = A(x(k)) x + B(x(k)) u \f$
	  at a specific point of the trajectory

	  \param threadId the id of the worker thread
	  \param k step k
	*/
    void computeLinearizedDynamics(size_t threadId, size_t k);


    //! Computes the linearized general constraints at a specific point of the trajectory
    /*!
	  This function calculates the linearization, i.e. matrices d, C and D in \f$ d_{lb} \leq C x + D u \leq d_{ub}\f$
	  at a specific point of the trajectory

	  \param threadId the id of the worker thread
	  \param k step k

	  \note the box constraints do not need to be linearized
	*/
    void computeLinearizedConstraints(size_t threadId, size_t k);

    //! Computes the quadratic costs
    /*!
	  This function calculates the quadratic costs as provided by the costFunction pointer.

	 * \param threadId id of worker thread
	 * \param k step k
	*/
    void computeQuadraticCosts(size_t threadId, size_t k);

    //! Initializes cost to go
    /*!
	 * This function initializes the cost-to-go function at time K.
     *
	 */
    void initializeCostToGo();

    //! Computes cost to go
    /*!
	 * This function computes the cost-to-go function for all times t<t_K
     *
	 * \param k step k
	 */
    void computeCostToGo(size_t k);

    //! Design controller
    /*!
	 * This function designes the LQR and feedforward controller at time k.
     *
	 * \param k step k
	 */
    void designController(size_t k);

    //! Compute cost for a given set of state and input trajectory
    /*!
	 * Compute cost for a given set of state and input trajectory
     *
	 * \param threadId the ID of the thread
	 * \param x_local the state trajectory
	 * \param u_local the control trajectory
	 * \param intermediateCost the accumulated intermediate cost
	 * \param finalCost the accumulated final cost
	 */
    void computeCostsOfTrajectory(size_t threadId,
        const core::StateVectorArray<STATE_DIM, SCALAR>& x_local,
        const core::ControlVectorArray<CONTROL_DIM, SCALAR>& u_local,
        scalar_t& intermediateCost,
        scalar_t& finalCost) const;

    /*!
	 * @brief Compute box constraint violations for a given set of state and input trajectory
     *
	 * \param threadId the ID of the thread
	 * \param x_local the state trajectory
	 * \param u_local the control trajectory
	 * \param e_tot the total accumulated box constraint violation
	 */
    void computeBoxConstraintErrorOfTrajectory(size_t threadId,
        const ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_local,
        const ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>& u_local,
        scalar_t& e_tot) const;

    /*!
	 * @brief Compute general constraint violations for a given set of state and input trajectory
     *
	 * \param threadId the ID of the thread
	 * \param x_local the state trajectory
	 * \param u_local the control trajectory
	 * \param e_tot the total accumulated general constraint violation
	 */
    void computeGeneralConstraintErrorOfTrajectory(size_t threadId,
        const ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_local,
        const ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>& u_local,
        scalar_t& e_tot) const;

    //! Check if controller with particular alpha is better
    void executeLineSearchSingleShooting(const size_t threadId,
        const scalar_t alpha,
        StateVectorArray& x_local,
        ControlVectorArray& u_local,
        scalar_t& intermediateCost,
        scalar_t& finalCost,
        scalar_t& e_box_norm,
        scalar_t& e_gen_norm,
        StateSubsteps& substepsX,
        ControlSubsteps& substepsU,
        std::atomic_bool* terminationFlag = nullptr) const;


    void executeLineSearchMultipleShooting(const size_t threadId,
        const scalar_t alpha,
        const ControlVectorArray& u_ff_update,
        const StateVectorArray& x_update,
        ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_recorded,
        ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_shot_recorded,
        ct::core::StateVectorArray<STATE_DIM, SCALAR>& defects_recorded,
        ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>& u_recorded,
        scalar_t& intermediateCost,
        scalar_t& finalCost,
        scalar_t& defectNorm,
        scalar_t& e_box_norm,
        scalar_t& e_gen_norm,
        StateSubsteps& substepsX,
        ControlSubsteps& substepsU,
        std::atomic_bool* terminationFlag = nullptr) const;


    //! Update feedforward controller
    /*!
	 * This function updates the feedforward Controller based on the previous calculation.
     *
	 * \param k step k
	 */
    void updateFFController(size_t k);


    //! Send a std::vector of Eigen to Matlab
    /*!
	 * This is a helper function to efficiently send std::vectors to Matlab.
	 */
    //    template <class V>
    //    void matrixToMatlab(V& matrix, std::string variableName);

    //! compute norm of a discrete array (todo move to core)
    template <typename ARRAY_TYPE, size_t ORDER = 1>
    SCALAR computeDiscreteArrayNorm(const ARRAY_TYPE& d) const;

    //! compute norm of difference between two discrete arrays (todo move to core)
    template <typename ARRAY_TYPE, size_t ORDER = 1>
    SCALAR computeDiscreteArrayNorm(const ARRAY_TYPE& a, const ARRAY_TYPE& b) const;

    //! compute the norm of the defects trajectory
    /*!
	 * Note that different kind of norms might be favorable for different cases.
	 * According to Nocedal and Wright, the l1-norm is "exact" (p.435),  the l2-norm is smooth.
	 */
    template <size_t ORDER = 1>
    SCALAR computeDefectsNorm(const StateVectorArray& d) const;

    typedef std::shared_ptr<ct::core::SubstepRecorder<STATE_DIM, CONTROL_DIM, SCALAR>> SubstepRecorderPtr;
    std::vector<SubstepRecorderPtr, Eigen::aligned_allocator<SubstepRecorderPtr>> substepRecorders_;

    typedef std::shared_ptr<ct::core::Integrator<STATE_DIM, SCALAR>> IntegratorPtr;
    std::vector<IntegratorPtr, Eigen::aligned_allocator<IntegratorPtr>> integrators_;  //! Runge-Kutta-4 Integrators

    typedef std::shared_ptr<Sensitivity_t> SensitivityPtr;
    std::vector<SensitivityPtr, Eigen::aligned_allocator<SensitivityPtr>>
        sensitivity_;  //! the ct sensitivity integrators

    typedef std::shared_ptr<ct::core::IntegratorSymplecticEuler<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>
        IntegratorSymplecticEulerPtr;
    std::vector<IntegratorSymplecticEulerPtr, Eigen::aligned_allocator<IntegratorSymplecticEulerPtr>>
        integratorsEulerSymplectic_;

    typedef std::shared_ptr<ct::core::IntegratorSymplecticRk<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>
        IntegratorSymplecticRkPtr;
    std::vector<IntegratorSymplecticRkPtr, Eigen::aligned_allocator<IntegratorSymplecticRkPtr>>
        integratorsRkSymplectic_;

    typedef std::shared_ptr<core::ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>> ConstantControllerPtr;
    std::vector<ConstantControllerPtr, Eigen::aligned_allocator<ConstantControllerPtr>>
        controller_;  //! the constant controller for forward-integration during one time-step


    //! The policy. currently only for returning the result, should eventually replace L_ and u_ff_ (todo)
    NLOCBackendBase::Policy_t policy_;

    ct::core::tpl::TimeArray<SCALAR> t_;  //! the time trajectory

    bool initialized_;
    bool configured_;

    size_t iteration_; /*!< current iteration */

    Settings_t settings_;

    int K_;  //! the number of stages in the overall OptConProblem

    StateVectorArray lx_;
    StateVectorArray x_;
    StateVectorArray xShot_;
    StateVectorArray x_prev_;

    ControlVectorArray lu_;
    ControlVectorArray u_ff_;
    ControlVectorArray u_ff_prev_;

    FeedbackArray L_;

    SCALAR d_norm_;      //! sum of the norms of all defects (internal constraint)
    SCALAR e_box_norm_;  //! sum of the norms of all box constraint violations
    SCALAR e_gen_norm_;  //! sum of the norms of all general constraint violations
    SCALAR lx_norm_;     //! sum of the norms of state update
    SCALAR lu_norm_;     //! sum of the norms of control update

    //! shared pointer to the linear-quadratic optimal control problem
    std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>> lqocProblem_;

    //! shared pointer to the linear-quadratic optimal control solver
    std::shared_ptr<LQOCSolver<STATE_DIM, CONTROL_DIM, SCALAR>> lqocSolver_;

    StateSubstepsPtr substepsX_;
    ControlSubstepsPtr substepsU_;

    scalar_t intermediateCostBest_;
    scalar_t finalCostBest_;
    scalar_t lowestCost_;

    //! costs of the previous iteration, required to determine convergence
    scalar_t intermediateCostPrevious_;
    scalar_t finalCostPrevious_;


//! if building with MATLAB support, include matfile
#ifdef MATLAB
    matlab::MatFile matFile_;
#endif  //MATLAB


    /*!
	 * of the following objects, we have nThreads+1 instantiations in form of a vector.
	 * Every instantiation is dedicated to a certain thread in the multi-thread implementation
	 */
    std::vector<typename OptConProblem_t::DynamicsPtr_t> systems_;
    std::vector<typename OptConProblem_t::LinearPtr_t> linearSystems_;
    std::vector<typename OptConProblem_t::CostFunctionPtr_t> costFunctions_;
    std::vector<typename OptConProblem_t::ConstraintPtr_t> boxConstraints_;
    std::vector<typename OptConProblem_t::ConstraintPtr_t> generalConstraints_;


    bool firstRollout_;
    scalar_t alphaBest_;

    //! a counter used to identify lqp problems in derived classes, i.e. for thread management in MP
    size_t lqpCounter_;

    SummaryAllIterations<SCALAR> summaryAllIterations_;
};


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
template <typename ARRAY_TYPE, size_t ORDER>
SCALAR NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeDiscreteArrayNorm(
    const ARRAY_TYPE& d) const
{
    SCALAR norm = 0.0;

    for (size_t k = 0; k < d.size(); k++)
    {
        norm += d[k].template lpNorm<ORDER>();
    }
    return norm;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
template <typename ARRAY_TYPE, size_t ORDER>
SCALAR NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeDiscreteArrayNorm(const ARRAY_TYPE& a,
    const ARRAY_TYPE& b) const
{
    assert(a.size() == b.size());

    SCALAR norm = 0.0;

    for (size_t k = 0; k < a.size(); k++)
    {
        norm += (a[k] - b[k]).template lpNorm<ORDER>();
    }
    return norm;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
template <size_t ORDER>
SCALAR NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeDefectsNorm(
    const StateVectorArray& d) const
{
    return computeDiscreteArrayNorm<StateVectorArray, ORDER>(d);
}


}  // namespace optcon
}  // namespace ct


#undef SYMPLECTIC_ENABLED
#undef SYMPLECTIC_DISABLED
