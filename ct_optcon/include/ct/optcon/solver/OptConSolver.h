/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


/**
 * OptConSolver.hpp
 *
 *
 * Requirements:
 * - returns an optimal controller. These can be different controller types, feedforward only, feedforward-feedback, feedback only,
 * 		therefore it is templated on the controller type
 */

#pragma once

#include <ct/optcon/problem/ContinuousOptConProblem.h>


namespace ct {
namespace optcon {


/** \defgroup OptConSolver OptConSolver
 * Solver interface for finite horizon optimal control problems
 *
 *  * Requirements:
 * - returns an optimal controller. These can be different controller types, feedforward only, feedforward-feedback, feedback only,
 * 		therefore it is templated on the controller type
 */
template <typename DERIVED,
    typename POLICY,
    typename SETTINGS,
    size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SCALAR = double,
    bool CONTINUOUS = true>
class OptConSolver
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename std::conditional<CONTINUOUS,
        ContinuousOptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>,
        DiscreteOptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>>::type OptConProblem_t;

    static const size_t STATE_D = STATE_DIM;
    static const size_t CONTROL_D = CONTROL_DIM;

    typedef POLICY Policy_t;
    typedef SETTINGS Settings_t;
    typedef DERIVED Derived;
    typedef SCALAR Scalar_t;

    OptConSolver() {}
    virtual ~OptConSolver() {}
    /*!
	 * \brief Assigns the optimal control problem to the solver.
	 *
	 * Most solvers will require some computational effort to adjust to a new problem.
	 * Therefore, when only adjusting a problem, it is more efficient to adjust individual
	 * properties instead of assigning an entirely new problem. To adjust properties, you can
	 * use
	 *  - changeTimeHorizon()
	 *  - changeInitialState()
	 *  - changeCostFunction()
	 *  - changeSystemDynamics()
	 *  - changeSystemLinearization()
	 *
	 * @return returns true if the optimal control structure is valid for the given solver
	 */
    virtual void setProblem(const OptConProblem_t& optConProblem)
    {
        optConProblem.verify();

        changeTimeHorizon(optConProblem.getTimeHorizon());
        changeInitialState(optConProblem.getInitialState());
        changeCostFunction(optConProblem.getCostFunction());
        changeNonlinearSystem(optConProblem.getNonlinearSystem());
        changeLinearSystem(optConProblem.getLinearSystem());

        if (optConProblem.getInputBoxConstraints())
            changeInputBoxConstraints(optConProblem.getInputBoxConstraints());
        if (optConProblem.getStateBoxConstraints())
            changeStateBoxConstraints(optConProblem.getStateBoxConstraints());
        if (optConProblem.getGeneralConstraints())
            changeGeneralConstraints(optConProblem.getGeneralConstraints());
    }

    /**
	 * configures the solver
	 * */
    virtual void configure(const Settings_t& settings) = 0;

    /**
	 * configures the solver from configFile
	 * */
    void configureFromFile(const std::string& filename,
        bool verbose = true,
        const std::string& ns = DERIVED::SolverName)
    {
        Settings_t settings;
        settings.load(filename, verbose, ns);
        configure(settings);
    }

    /**
	 * solve the optimal control problem
	 * @return true if solve succeeded, false otherwise.
	 * */
    virtual bool solve() = 0;

    /**
	 * run a single iteration of the solver (might not be supported by all solvers)
	 * @return true if a better solution was found
	 */
    virtual bool runIteration() { throw std::runtime_error("runIteration not supported by solver"); }
    /**
	 * Get the optimized control policy to the optimal control problem
	 * @return
	 */
    virtual const Policy_t& getSolution() = 0;

    /**
	 * Get the optimized trajectory to the optimal control problem
	 * @return
	 */
    virtual const core::StateTrajectory<STATE_DIM, SCALAR> getStateTrajectory() const = 0;

    /**
	 * Get the optimal feedforward control input corresponding to the optimal trajectory
	 * @return
	 */
    virtual const core::ControlTrajectory<CONTROL_DIM, SCALAR> getControlTrajectory() const = 0;

    /**
	 * Get the time indices corresponding to the solution
	 * @return
	 */
    virtual const core::tpl::TimeArray<SCALAR>& getTimeArray() const = 0;


    /*!
	 * Set the initial guess used by the solver (not all solvers might support initial guesses)
	 */
    virtual void setInitialGuess(const Policy_t& initialGuess) = 0;


    /*!
	 * \brief Get the time horizon the solver currently operates on.
	 *
	 */
    virtual SCALAR getTimeHorizon() const = 0;


    /*!
	 * \brief Change the time horizon the solver operates on.
	 *
	 * This function does not need to be called if setProblem() has been called
	 * with an OptConProblem that had the correct time horizon set.
	 */
    virtual void changeTimeHorizon(const SCALAR& tf) = 0;

    /*!
	 * \brief Change the initial state for the optimal control problem
	 *
	 * This function does not need to be called if setProblem() has been called
	 * with an OptConProblem that had the correct initial state set
	 */
    virtual void changeInitialState(const core::StateVector<STATE_DIM, SCALAR>& x0) = 0;

    /*!
	 * \brief Change the cost function
	 *
	 * This function does not need to be called if setProblem() has been called
	 * with an OptConProblem that had the correct cost function
	 */
    virtual void changeCostFunction(const typename OptConProblem_t::CostFunctionPtr_t& cf) = 0;

    /*!
	 * \brief Change the nonlinear system
	 *
	 * This function does not need to be called if setProblem() has been called
	 * with an OptConProblem that had the correct nonlinear system
	 */
    virtual void changeNonlinearSystem(const typename OptConProblem_t::DynamicsPtr_t& dyn) = 0;

    /*!
	 * \brief Change the linear system
	 *
	 * This function does not need to be called if setProblem() has been called
	 * with an OptConProblem that had the correct linear system
	 */
    virtual void changeLinearSystem(const typename OptConProblem_t::LinearPtr_t& lin) = 0;

    /**
	 * @brief      Change the box constraints
	 *
	 *             This function does not need to be called if
	 *             setProblem() has been called with an OptConProblem that
	 *             had the correct linear system
	 *
	 * @param[in]  con   The new box constraints
	 */
    virtual void changeInputBoxConstraints(const typename OptConProblem_t::ConstraintPtr_t con)
    {
        throw std::runtime_error("The current solver does not support input box constraints!");
    }

    virtual void changeStateBoxConstraints(const typename OptConProblem_t::ConstraintPtr_t con)
    {
        throw std::runtime_error("The current solver does not support state box constraints!");
    }

    /**
	 * @brief      Change the general constraints.
	 *
	 *             This function does not need to be called if
	 *             setProblem() has been called with an OptConProblem that
	 *             had the correct linear system
	 *
	 * @param[in]  con   The new general constraints
	 */
    virtual void changeGeneralConstraints(const typename OptConProblem_t::ConstraintPtr_t con)
    {
        throw std::runtime_error("The current solver does not support general constraints!");
    }

    virtual SCALAR getCost() const { throw std::runtime_error("Get cost not implemented"); }
    /*!
	 * \brief Direct accessor to the system instances
	 *
	 * \warning{Use this only when performance absolutely matters and if you know what you
	 * are doing. Otherwise use e.g. changeNonlinearSystem() to change the system dynamics
	 * in a safe and easy way. You should especially not change the size of the vector or
	 * modify each entry differently.}
	 * @return
	 */
    virtual std::vector<typename OptConProblem_t::DynamicsPtr_t>& getNonlinearSystemsInstances() = 0;

    virtual const std::vector<typename OptConProblem_t::DynamicsPtr_t>& getNonlinearSystemsInstances() const = 0;

    /*!
	 * \brief Direct accessor to the linear system instances
	 *
	 * \warning{Use this only when performance absolutely matters and if you know what you
	 * are doing. Otherwise use e.g. changeLinearSystem() to change the system dynamics
	 * in a safe and easy way. You should especially not change the size of the vector or
	 * modify each entry differently.}
	 * @return
	 */
    virtual std::vector<typename OptConProblem_t::LinearPtr_t>& getLinearSystemsInstances() = 0;

    virtual const std::vector<typename OptConProblem_t::LinearPtr_t>& getLinearSystemsInstances() const = 0;

    /*!
	 * \brief Direct accessor to the cost function instances
	 *
	 * \warning{Use this only when performance absolutely matters and if you know what you
	 * are doing. Otherwise use e.g. changeCostFunction() to change the system dynamics
	 * in a safe and easy way. You should especially not change the size of the vector or
	 * modify each entry differently.}
	 * @return
	 */
    virtual std::vector<typename OptConProblem_t::CostFunctionPtr_t>& getCostFunctionInstances() = 0;

    virtual const std::vector<typename OptConProblem_t::CostFunctionPtr_t>& getCostFunctionInstances() const = 0;

    /**
	 * @brief      Direct accessor to the box constraint instances
	 *
	 * \warning{Use this only when performance absolutely matters and if you know what you
	 * are doing. Otherwise use e.g. changeCostFunction() to change the system dynamics
	 * in a safe and easy way. You should especially not change the size of the vector or
	 * modify each entry differently.}
	 *
	 * @return     The state box constraint instances
	 */
    virtual std::vector<typename OptConProblem_t::ConstraintPtr_t>& getInputBoxConstraintsInstances()
    {
        throw std::runtime_error("getInputBoxConstraintsInstances not supported.");
    }

    virtual const std::vector<typename OptConProblem_t::ConstraintPtr_t>& getInputBoxConstraintsInstances() const
    {
        throw std::runtime_error("getInputBoxConstraintsInstances not supported.");
    }

    virtual std::vector<typename OptConProblem_t::ConstraintPtr_t>& getStateBoxConstraintsInstances()
    {
        throw std::runtime_error("getStateBoxConstraintsInstances not supported.");
    }

    virtual const std::vector<typename OptConProblem_t::ConstraintPtr_t>& getStateBoxConstraintsInstances() const
    {
        throw std::runtime_error("getStateBoxConstraintsInstances not supported.");
    }


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
    virtual std::vector<typename OptConProblem_t::ConstraintPtr_t>& getGeneralConstraintsInstances()
    {
        throw std::runtime_error("getGeneralConstraintsInstances not supported.");
    }

    virtual const std::vector<typename OptConProblem_t::ConstraintPtr_t>& getGeneralConstraintsInstances() const
    {
        throw std::runtime_error("getGeneralConstraintsInstances not supported.");
    }

#ifdef CPPADCG
    /**
	 * @brief      Generates and compiles AD source code which can be used in
	 *             the solver. This method can be called just before solving the
	 *             problem, i.e. it can be called from the same executable than
	 *             the actual solve
	 *
	 * @param[in]  problemCG  The optcon problem templated on the AD CG Scalar
	 * @param[in]  settings   The settings indicating what to generate
	 */
    virtual void generateAndCompileCode(
        const ContinuousOptConProblem<STATE_DIM, CONTROL_DIM, ct::core::ADCGScalar>& problemCG,
        const ct::core::DerivativesCppadSettings& settings)
    {
        throw std::runtime_error("Generate and compile code not implemented for this solver");
    }
#endif

    /**
	 * @brief      Generates source AD source code which can be used in the
	 *             solver. This method needs to be called ahead of actually
	 *             solving the problem (e.g. from a different executable)
	 *
	 * @param[in]  settings  The settings indicating what to generate
	 */
    virtual void generateCode(const ct::core::DerivativesCppadSettings& settings)
    {
        throw std::runtime_error("Generate Code not implemented for this solver");
    }
};
}  // namespace optcon
}  // namespace ct
