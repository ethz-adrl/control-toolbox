/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#pragma once

#include <ct/optcon/problem/OptConProblem.h>

#include <ct/optcon/nloc/NLOCBackendST.hpp>
#include <ct/optcon/nloc/NLOCBackendMP.hpp>

#include <ct/optcon/nloc/algorithms/ilqr/iLQR.hpp>
#include <ct/optcon/nloc/algorithms/gnms/GNMS.hpp>

namespace ct {
namespace optcon {


/** \defgroup OptConSolver OptConSolver
 * Solver interface for finite horizon optimal control problems
 */
template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    size_t P_DIM = STATE_DIM / 2,
    size_t V_DIM = STATE_DIM / 2,
    typename SCALAR = double>
class NLOptConSolver : public OptConSolver<NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>,
                           core::StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>,
                           NLOptConSettings,
                           STATE_DIM,
                           CONTROL_DIM,
                           SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_D = STATE_DIM;
    static const size_t CONTROL_D = CONTROL_DIM;
    static const size_t POS_DIM = P_DIM;
    static const size_t VEL_DIM = V_DIM;

    typedef NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR> Derived;
    typedef core::StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR> Policy_t;
    typedef NLOptConSettings Settings_t;
    typedef SCALAR Scalar_t;
    typedef OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR> OptConProblem_t;


    //! constructor
    NLOptConSolver(const OptConProblem_t& optConProblem, const Settings_t& settings);

    //! constructor
    NLOptConSolver(const OptConProblem_t& optConProblem,
        const std::string& settingsFile,
        bool verbose = true,
        const std::string& ns = "alg");

    //! destructor
    virtual ~NLOptConSolver();

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
    virtual bool runIteration();

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
    const std::shared_ptr<NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>>& getBackend();

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

    //! get reference to the state input constraints
    std::vector<typename OptConProblem_t::ConstraintPtr_t>& getStateInputConstraintsInstances() override;

    //! get constant reference to the state input constraints
    const std::vector<typename OptConProblem_t::ConstraintPtr_t>& getStateInputConstraintsInstances() const override;

    //! get reference to the pure state constraints
    std::vector<typename OptConProblem_t::ConstraintPtr_t>& getPureStateConstraintsInstances() override;

    //! get constant reference to the pure state constraints
    const std::vector<typename OptConProblem_t::ConstraintPtr_t>& getPureStateConstraintsInstances() const override;

    //! logging a short summary to matlab
    void logSummaryToMatlab(const std::string& fileName);

protected:
    //! the backend holding all the math operations
    std::shared_ptr<NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>> nlocBackend_;

    //! the algorithm for sequencing the math operations in correct manner
    std::shared_ptr<NLOCAlgorithm<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>> nlocAlgorithm_;
};


}  // namespace optcon
}  // namespace ct
