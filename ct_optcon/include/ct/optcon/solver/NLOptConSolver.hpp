/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/optcon/problem/ContinuousOptConProblem.h>

#include <ct/optcon/nloc/NLOCBackendST.hpp>
#include <ct/optcon/nloc/NLOCBackendMP.hpp>

#include <ct/optcon/nloc/algorithms/SingleShooting.hpp>
#include <ct/optcon/nloc/algorithms/MultipleShooting.hpp>

namespace ct {
namespace optcon {


/** \defgroup OptConSolver OptConSolver
 * Solver interface for finite horizon optimal control problems
 */
template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    size_t P_DIM = STATE_DIM / 2,
    size_t V_DIM = STATE_DIM / 2,
    typename SCALAR = double,
    bool CONTINUOUS = true>
class NLOptConSolver final
    : public OptConSolver<NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>,
          typename NLOCAlgorithm<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::Policy_t,
          NLOptConSettings,
          STATE_DIM,
          CONTROL_DIM,
          SCALAR,
          CONTINUOUS>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_D = STATE_DIM;
    static const size_t CONTROL_D = CONTROL_DIM;
    static const size_t POS_DIM = P_DIM;
    static const size_t VEL_DIM = V_DIM;

    typedef OptConSolver<NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>,
        typename NLOCAlgorithm<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::Policy_t,
        NLOptConSettings,
        STATE_DIM,
        CONTROL_DIM,
        SCALAR,
        CONTINUOUS>
        Base;


    typedef NLOptConSettings Settings_t;
    typedef SCALAR Scalar_t;

    typedef typename Base::OptConProblem_t OptConProblem_t;

    typedef NLOCAlgorithm<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS> NLOCAlgorithm_t;
    typedef typename NLOCAlgorithm_t::Policy_t Policy_t;
    typedef typename NLOCAlgorithm_t::Backend_t Backend_t;


    //! constructor
    NLOptConSolver(const OptConProblem_t& optConProblem, const Settings_t& settings);

    //! constructor
    NLOptConSolver(const OptConProblem_t& optConProblem,
        const std::string& settingsFile,
        bool verbose = true,
        const std::string& ns = "alg");

    //! destructor
    virtual ~NLOptConSolver() = default;

    /**
	 * configures the solver
	 * */
    void initialize(const OptConProblem_t& optConProblem, const Settings_t& settings);

    /**
	 * configures the solver
	 * */
    void configure(const Settings_t& settings) override;


    /*!
	 * execute preparation steps for an iteration, e.g. computation of defects
	 */
    virtual void prepareIteration();

    /*!
	 * execute finishing step for an iteration, e.g. solving Riccati backward pass.
	 * @return
	 */
    virtual bool finishIteration();


    /*!
	 * execute preparation steps for an iteration, e.g. computation of defects
	 */
    virtual void prepareMPCIteration();

    /*!
	 * execute finishing step for an iteration, e.g. solving Riccati backward pass.
	 * @return
	 */
    virtual bool finishMPCIteration();

    /**
	 * run a single iteration of the solver
	 * @return true if a better solution was found
	 */
    virtual bool runIteration() override;

    /*!
	 * Set the initial guess used by the solver (not all solvers might support initial guesses)
	 */
    void setInitialGuess(const Policy_t& initialGuess) override;

    /**
	 * solve the optimal control problem
	 * */
    virtual bool solve() override;

    /**
	 * Get the optimized control policy to the optimal control problem
	 * @return
	 */
    virtual const Policy_t& getSolution() override;

    /**
	 * Get the optimized trajectory to the optimal control problem
	 * @return
	 */
    virtual const core::StateTrajectory<STATE_DIM, SCALAR> getStateTrajectory() const override;

    /**
	 * Get the optimal feedforward control input corresponding to the optimal trajectory
	 * @return
	 */
    virtual const core::ControlTrajectory<CONTROL_DIM, SCALAR> getControlTrajectory() const override;

    /**
	 * Get the time indices corresponding to the solution
	 * @return
	 */
    virtual const core::tpl::TimeArray<SCALAR>& getTimeArray() const override;

    /*!
	 * \brief Get the time horizon the solver currently operates on.
	 *
	 */
    virtual SCALAR getTimeHorizon() const override;

    /*!
	 * \brief Change the time horizon the solver operates on.
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct time horizon set.
	 */
    virtual void changeTimeHorizon(const SCALAR& tf) override;

    /*!
	 * \brief Change the initial state for the optimal control problem
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct initial state set
	 */
    virtual void changeInitialState(const core::StateVector<STATE_DIM, SCALAR>& x0) override;

    /*!
	 * \brief Change the cost function
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct cost function
	 */
    virtual void changeCostFunction(const typename OptConProblem_t::CostFunctionPtr_t& cf) override;

    /*!
	 * \brief Change the nonlinear system
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct nonlinear system
	 */
    virtual void changeNonlinearSystem(const typename OptConProblem_t::DynamicsPtr_t& dyn) override;

    /*!
	 * \brief Change the linear system
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct linear system
	 */
    virtual void changeLinearSystem(const typename OptConProblem_t::LinearPtr_t& lin) override;

    virtual SCALAR getCost() const override;

    //! get a reference to the current settings
    const Settings_t& getSettings();

    //! get a reference to the backend (@todo this is not optimal, allows the user too much access)
    const std::shared_ptr<Backend_t>& getBackend();

    //! get reference to the nonlinear system
    std::vector<typename OptConProblem_t::DynamicsPtr_t>& getNonlinearSystemsInstances() override;

    //! get constant reference to the nonlinear system
    const std::vector<typename OptConProblem_t::DynamicsPtr_t>& getNonlinearSystemsInstances() const override;

    //! get reference to the linearized system
    std::vector<typename OptConProblem_t::LinearPtr_t>& getLinearSystemsInstances() override;

    //! get constant reference to the linearized system
    const std::vector<typename OptConProblem_t::LinearPtr_t>& getLinearSystemsInstances() const override;

    //! get reference to the cost function
    std::vector<typename OptConProblem_t::CostFunctionPtr_t>& getCostFunctionInstances() override;

    //! get constant reference to the cost function
    const std::vector<typename OptConProblem_t::CostFunctionPtr_t>& getCostFunctionInstances() const override;

    //! get reference to the box constraints
    std::vector<typename OptConProblem_t::ConstraintPtr_t>& getInputBoxConstraintsInstances() override;

    //! get constant reference to the boxconstraints
    const std::vector<typename OptConProblem_t::ConstraintPtr_t>& getInputBoxConstraintsInstances() const override;

    //! get reference to the box constraints
    std::vector<typename OptConProblem_t::ConstraintPtr_t>& getStateBoxConstraintsInstances() override;

    //! get constant reference to the boxconstraints
    const std::vector<typename OptConProblem_t::ConstraintPtr_t>& getStateBoxConstraintsInstances() const override;

    //! get reference to the general constraints
    std::vector<typename OptConProblem_t::ConstraintPtr_t>& getGeneralConstraintsInstances() override;

    //! get constant reference to the general constraints
    const std::vector<typename OptConProblem_t::ConstraintPtr_t>& getGeneralConstraintsInstances() const override;

    //! logging a short summary to matlab
    void logSummaryToMatlab(const std::string& fileName);

protected:
    //! the backend holding all the math operations
    std::shared_ptr<Backend_t> nlocBackend_;

    //! the algorithm for sequencing the math operations in correct manner
    std::shared_ptr<NLOCAlgorithm_t> nlocAlgorithm_;

private:
    //! set algorithm, use as private only
    void setAlgorithm(const Settings_t& settings);
};


}  // namespace optcon
}  // namespace ct
