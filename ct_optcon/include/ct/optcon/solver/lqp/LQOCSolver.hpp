/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/optcon/solver/NLOptConSettings.hpp>

#include <ct/optcon/problem/LQOCProblem.hpp>

namespace ct {
namespace optcon {

/*!
 * Base class for solvers to solve an LQOCProblem
 * (both constrained / unconstrained, etc.)
 *
 * \todo uncouple from NLOptConSettings
 */
template <typename MANIFOLD, size_t CONTROL_DIM>
class LQOCSolver
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr size_t STATE_DIM = MANIFOLD::TangentDim;
    using SCALAR = typename MANIFOLD::Scalar;

    typedef LQOCProblem<MANIFOLD, CONTROL_DIM> LQOCProblem_t;

    /*!
	 * Constructor. Initialize by handing over an LQOCProblem, or otherwise by calling setProblem()
	 * @param lqocProblem shared_ptr to the LQOCProblem to be solved.
	 */
    LQOCSolver(const std::shared_ptr<LQOCProblem_t>& lqocProblem = nullptr) : lqocProblem_(lqocProblem) {}
    virtual ~LQOCSolver() = default;

    /*!
	 * set a new problem
	 * update the shared_ptr to the LQOCProblem instance and call initialize instance deriving from this class.
	 * @param lqocProblem
	 */
    void setProblem(std::shared_ptr<LQOCProblem_t> lqocProblem)
    {
        setProblemImpl(lqocProblem);
        lqocProblem_ = lqocProblem;
    }

    virtual void configure(const NLOptConSettings& settings) = 0;

    //! setup and configure the box constraints
    // return true if configuration changed, otherwise false
    virtual bool configureInputBoxConstraints(std::shared_ptr<LQOCProblem_t> lqocProblem)
    {
        throw std::runtime_error("input box constraints are not available for this solver.");
    }

    // return true if configuration changed, otherwise false
    virtual bool configureStateBoxConstraints(std::shared_ptr<LQOCProblem_t> lqocProblem)
    {
        throw std::runtime_error("state box constraints are not available for this solver.");
    }

    //! setup and configure the general (in)equality constraints
    // return true if configuration changed, otherwise false
    virtual bool configureGeneralConstraints(std::shared_ptr<LQOCProblem_t> lqocProblem)
    {
        throw std::runtime_error("general constraints are not available for this solver.");
    }

    //! a method reserved for memory allocation (e.g. required for HPIPM)
    virtual void initializeAndAllocate() = 0;

    //! solve the LQOC problem
    virtual void solve() = 0;

    virtual void solveSingleStage(int N)
    {
        throw std::runtime_error("solveSingleStage not available for this solver.");
    }

    //! extract the solution (can be overriden if additional extraction steps required in specific solver)
    virtual void computeStatesAndControls() = 0;
    //! return solution for state
    const core::DiscreteArray<typename MANIFOLD::Tangent>& getSolutionState() { return x_sol_; }
    //! return solution for control
    const ct::core::ControlVectorArray<SCALAR>& getSolutionControl() { return u_sol_; }
    //! return TVLQR feedback matrices
    virtual void computeFeedbackMatrices() = 0;
    const ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR>& getSolutionFeedback() { return L_; }
    //! compute iLQR-style lv
    virtual void compute_lv() = 0;
    //! return iLQR-style feedforward lv
    virtual const ct::core::ControlVectorArray<SCALAR>& get_lv() { return lv_; }
    //! return the smallest eigenvalue
    virtual SCALAR getSmallestEigenvalue()
    {
        throw std::runtime_error("getSmallestEigenvalue not available for this solver.");
    }


protected:
    virtual void setProblemImpl(std::shared_ptr<LQOCProblem_t> lqocProblem) = 0;

    std::shared_ptr<LQOCProblem_t> lqocProblem_;

    core::DiscreteArray<typename MANIFOLD::Tangent> x_sol_;            // solution in x
    core::ControlVectorArray<SCALAR> u_sol_;        // solution in u
    ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR> L_;  // solution feedback
    ct::core::ControlVectorArray<SCALAR> lv_;       // feedforward increment (iLQR-style)
};

}  // namespace optcon
}  // namespace ct
