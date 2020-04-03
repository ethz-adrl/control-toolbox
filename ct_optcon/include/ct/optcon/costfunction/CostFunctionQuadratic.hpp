/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "CostFunction.hpp"
#include "term/TermBase.hpp"

namespace ct {
namespace optcon {


/**
 * \ingroup CostFunction
 *
 * \brief Describes a cost function with a quadratic approximation, i.e. one that
 * can compute first and second order derivatives with respect to state and
 * control input. **This does NOT mean it has to be a purely quadratic cost
 * function**. If you are looking for a purely quadratic cost function, check
 * CostFunctionQuadraticSimple.
 *
 * A cost function is assumed to be a sum of intermediate and final terms, i.e.
 * \f$ J(x,u,t) = \sum_{n=0}^{N_i} T_{i,n}(x,u,t) + \sum_{n=0}^{N_f} T_{i,f}(x,u,t) \f$
 * These terms can have arbitrary form.
 */
template <typename MANIFOLD, size_t CONTROL_DIM>
class CostFunctionQuadratic : public CostFunction<MANIFOLD, CONTROL_DIM>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr size_t STATE_DIM = MANIFOLD::TangentDim;
    using SCALAR_EVAL = typename MANIFOLD::Scalar;  // TODO: is there a better name than scalar_eval?
    using Tangent_t = typename MANIFOLD::Tangent;

    using Base = CostFunction<MANIFOLD, CONTROL_DIM>;
    using control_vector_t = typename Base::control_vector_t;
    typedef ct::core::StateMatrix<STATE_DIM, SCALAR_EVAL> state_matrix_t;
    typedef ct::core::ControlMatrix<CONTROL_DIM, SCALAR_EVAL> control_matrix_t;
    typedef ct::core::ControlStateMatrix<STATE_DIM, CONTROL_DIM, SCALAR_EVAL> control_state_matrix_t;


    /**
	 * Constructor
	 */
    CostFunctionQuadratic();

    /**
	 * Copy constructor
	 * @param arg other cost function
	 */
    CostFunctionQuadratic(const CostFunctionQuadratic& arg);

    /**
	 * Clones the cost function.
	 * @return
	 */
    virtual CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>* clone() const = 0;

    /**
	 * Destructor
	 */
    virtual ~CostFunctionQuadratic();

    /**
	 * \brief Adds an intermediate term
	 * @param term intermediate term
	 * @param verbose verbosity flag which enables printout
	 * @return
	 */
    virtual size_t addIntermediateTerm(std::shared_ptr<TermBase<MANIFOLD, CONTROL_DIM>> term, bool verbose = false);

    /**
	 * \brief Adds a final term
	 * @param term final term
	 * @param verbose verbosity flag which enables printout
	 * @return
	 */
    virtual size_t addFinalTerm(std::shared_ptr<TermBase<MANIFOLD, CONTROL_DIM>> term, bool verbose = false);

    /**
	 * \brief Loads cost function from config file
	 * @param filename config file location
	 * @param verbose verbosity flag which enables printout
	 */
    virtual void loadFromConfigFile(const std::string& filename, bool verbose = false);

    /**
	 * \brief Computes intermediate-cost first-order derivative with respect to state
	 * @return derivative vector (Jacobian)
	 */
    virtual ct::core::StateVector<STATE_DIM, SCALAR_EVAL> stateDerivativeIntermediate() = 0;

    /**
	 * Computes terminal-cost first-order derivative with respect to state
	 * @return derivative vector (Jacobian)
	 */
    virtual ct::core::StateVector<STATE_DIM, SCALAR_EVAL> stateDerivativeTerminal() = 0;

    /**
	 * \brief Computes intermediate-cost second-order derivative with respect to state
	 * @return derivative matrix (Jacobian)
	 */
    virtual state_matrix_t stateSecondDerivativeIntermediate() = 0;

    /**
	 * \brief Computes final-cost second-order derivative with respect to state
	 * @return derivative matrix (Jacobian)
	 */
    virtual state_matrix_t stateSecondDerivativeTerminal() = 0;

    /**
	 * \brief Computes intermediate-cost first-order derivative with respect to control
	 * @return derivative vector (Jacobian)
	 */
    virtual control_vector_t controlDerivativeIntermediate() = 0;

    /**
	 * \brief Computes terminal-cost first-order derivative with respect to control
	 *
	 * Not available for all cost functions. Throws an exception if not available.
	 * @return derivative vector (Jacobian)
	 */
    virtual control_vector_t controlDerivativeTerminal();
    /**
	 * \brief Computes intermediate-cost second-order derivative with respect to input
	 * @return derivative matrix (Jacobian)
	 */
    virtual control_matrix_t controlSecondDerivativeIntermediate() = 0;

    /**
	 * \brief Computes final-cost second-order derivative with respect to input
	 *
	 * Not available for all cost functions. Throws an exception if not available.
	 * @return derivative matrix (Jacobian)
	 */
    virtual control_matrix_t controlSecondDerivativeTerminal();

    /**
	 * \brief Computes intermediate-cost derivative with respect to state and control
	 * @return derivative matrix (Jacobian)
	 */
    virtual control_state_matrix_t stateControlDerivativeIntermediate() = 0;

    /**
	 * \brief Computes final-cost derivative with respect to state and control
	 * @return derivative matrix (Jacobian)
	 */
    virtual control_state_matrix_t stateControlDerivativeTerminal();

    //! update the reference state for intermediate cost terms
    virtual void updateReferenceState(const MANIFOLD& x_ref);

    //! update the reference state for final cost terms
    virtual void updateFinalState(const MANIFOLD& x_final);

    //! update the reference control for intermediate cost terms
    virtual void updateReferenceControl(const control_vector_t& u_ref);

    //! compare the state derivative against numerical differentiation
    bool stateDerivativeIntermediateTest(bool verbose = false);

    //! compare the control derivative against numerical differentiation
    bool controlDerivativeIntermediateTest(bool verbose = false);

    std::shared_ptr<TermBase<MANIFOLD, CONTROL_DIM>> getIntermediateTermById(const size_t id);

    std::shared_ptr<TermBase<MANIFOLD, CONTROL_DIM>> getFinalTermById(const size_t id);

    std::shared_ptr<TermBase<MANIFOLD, CONTROL_DIM>> getIntermediateTermByName(const std::string& name);

    std::shared_ptr<TermBase<MANIFOLD, CONTROL_DIM>> getFinalTermByName(const std::string& name);

    //! initialize the cost function (e.g. to be used in CostFunctionAD)
    virtual void initialize();

protected:
    //! evaluate intermediate analytical cost terms
    SCALAR_EVAL evaluateIntermediateBase();

    //! evaluate terminal analytical cost terms
    SCALAR_EVAL evaluateTerminalBase();

    //! evaluate intermediate analytical state derivatives
    Tangent_t stateDerivativeIntermediateBase();

    //! evaluate terminal analytical state derivatives
    Tangent_t stateDerivativeTerminalBase();

    //! evaluate intermediate analytical state second derivatives
    state_matrix_t stateSecondDerivativeIntermediateBase();

    //! evaluate terminal analytical state second derivatives
    state_matrix_t stateSecondDerivativeTerminalBase();

    //! evaluate intermediate analytical control derivatives
    control_vector_t controlDerivativeIntermediateBase();

    //! evaluate terminal analytical control derivatives
    control_vector_t controlDerivativeTerminalBase();

    //! evaluate intermediate analytical control second derivatives
    control_matrix_t controlSecondDerivativeIntermediateBase();

    //! evaluate terminal analytical control second derivatives
    control_matrix_t controlSecondDerivativeTerminalBase();

    //! evaluate intermediate analytical control mixed state control derivatives
    control_state_matrix_t stateControlDerivativeIntermediateBase();

    //! evaluate terminal analytical control mixed state control derivatives
    control_state_matrix_t stateControlDerivativeTerminalBase();

    // TODO: this is codesmell .... somewhat not nice that this is here
    //! compute the state derivative by numerical differentiation (can be used for testing)
    // euclidean case
    template <typename M = MANIFOLD, typename std::enable_if<ct::core::is_euclidean<M>::value, bool>::type>
    Tangent_t stateDerivativeIntermediateNumDiff();
    // non-euclidean case
    template <typename M = MANIFOLD, typename std::enable_if<!(ct::core::is_euclidean<M>::value), bool>::type>
    Tangent_t stateDerivativeIntermediateNumDiff();

    //! compute the control derivative by numerical differentiation (can be used for testing)
    // euclidean case
    template <typename M = MANIFOLD, typename std::enable_if<ct::core::is_euclidean<M>::value, bool>::type>
    control_vector_t controlDerivativeIntermediateNumDiff();
    // non-euclidean case
    template <typename M = MANIFOLD, typename std::enable_if<!(ct::core::is_euclidean<M>::value), bool>::type>
    control_vector_t controlDerivativeIntermediateNumDiff();

    //! stepsize for numerical differentiation
    SCALAR_EVAL eps_;

    //! use double sided derivatives in numerical differentiation
    bool doubleSidedDerivative_ = true;

    /** list of intermediate cost terms for which analytic derivatives are available */
    std::vector<std::shared_ptr<TermBase<MANIFOLD, CONTROL_DIM>>> intermediateCostAnalytical_;

    /** list of final cost terms for which analytic derivatives are available */
    std::vector<std::shared_ptr<TermBase<MANIFOLD, CONTROL_DIM>>> finalCostAnalytical_;
};


// impl for euclidean case (do not move to *-impl.h file)
template <typename MANIFOLD, size_t CONTROL_DIM>
template <typename M, typename std::enable_if<ct::core::is_euclidean<M>::value, bool>::type>
auto CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::stateDerivativeIntermediateNumDiff() -> Tangent_t
{
    using state_vector_t = ct::core::StateVector<STATE_DIM, SCALAR_EVAL>;

    // TODO: make the numdiff linearizers generic and move them to ct_core
    state_vector_t dFdx = state_vector_t::Zero();
    state_vector_t x_local;
    control_vector_t u_local;
    SCALAR_EVAL t_local;
    this->getCurrentStateAndControl(x_local, u_local, t_local);
    SCALAR_EVAL dxdt_ref = this->evaluateIntermediate();

    for (size_t i = 0; i < STATE_DIM; ++i)
    {
        // inspired from http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
        SCALAR_EVAL h = eps_ * std::max(std::abs(x_local(i)), 1.0);
        volatile SCALAR_EVAL x_ph = x_local(i) + h;
        SCALAR_EVAL dxp = x_ph - x_local(i);

        state_vector_t x_perturbed = x_local;
        x_perturbed(i) = x_ph;

        // get evaluation of f(x,u)
        this->setCurrentStateAndControl(x_perturbed, u_local, t_local);
        SCALAR_EVAL dxdt = this->evaluateIntermediate();

        if (doubleSidedDerivative_)
        {
            SCALAR_EVAL dxdt_low;

            volatile SCALAR_EVAL x_mh = x_local(i) - h;
            SCALAR_EVAL dxm = x_local(i) - x_mh;

            x_perturbed = x_local;
            x_perturbed(i) = x_mh;
            this->setCurrentStateAndControl(x_perturbed, u_local, t_local);
            dxdt_low = this->evaluateIntermediate();
            dFdx(i, 0) = (dxdt - dxdt_low) / (dxp + dxm);
        }
        else
        {
            dFdx(i, 0) = (dxdt - dxdt_ref) / dxp;
        }
    }

    return dFdx;
}
// impl for manifold case (do not move to *-impl.h file)
template <typename MANIFOLD, size_t CONTROL_DIM>
template <typename M, typename std::enable_if<!(ct::core::is_euclidean<M>::value), bool>::type>
auto CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::stateDerivativeIntermediateNumDiff() -> Tangent_t
{
    throw std::runtime_error("stateDerivativeIntermediateNumDiff not implemented for manifold case.");
}

// impl for euclidean case (do not move to *-impl.h file)
template <typename MANIFOLD, size_t CONTROL_DIM>
template <typename M, typename std::enable_if<ct::core::is_euclidean<M>::value, bool>::type>
auto CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::controlDerivativeIntermediateNumDiff() -> control_vector_t
{
    using state_vector_t = ct::core::StateVector<STATE_DIM, SCALAR_EVAL>;

    control_vector_t dFdu = control_vector_t::Zero();
    state_vector_t x_local;
    control_vector_t u_local;
    SCALAR_EVAL t_local;
    this->getCurrentStateAndControl(x_local, u_local, t_local);
    SCALAR_EVAL dxdt_ref = this->evaluateIntermediate();

    for (size_t i = 0; i < CONTROL_DIM; ++i)
    {
        // inspired from http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
        SCALAR_EVAL h = eps_ * std::max(std::abs(u_local(i)), 1.0);
        volatile SCALAR_EVAL u_ph = u_local(i) + h;
        SCALAR_EVAL dup = u_ph - u_local(i);

        control_vector_t u_perturbed = u_local;
        u_perturbed(i) = u_ph;

        // get evaluation of f(x,u)
        this->setCurrentStateAndControl(x_local, u_perturbed, t_local);
        SCALAR_EVAL dxdt = this->evaluateIntermediate();

        if (doubleSidedDerivative_)
        {
            SCALAR_EVAL dxdt_low;

            volatile SCALAR_EVAL u_mh = u_local(i) - h;
            SCALAR_EVAL dum = u_local(i) - u_mh;

            u_perturbed = u_local;
            u_perturbed(i) = u_mh;
            this->setCurrentStateAndControl(x_local, u_perturbed, t_local);
            dxdt_low = this->evaluateIntermediate();

            dFdu(i, 0) = (dxdt - dxdt_low) / (dup + dum);
        }
        else
        {
            dFdu(i, 0) = (dxdt - dxdt_ref) / dup;
        }
    }

    return dFdu;
}
// impl for manifold case (do not move to *-impl.h file)
template <typename MANIFOLD, size_t CONTROL_DIM>
template <typename M, typename std::enable_if<!(ct::core::is_euclidean<M>::value), bool>::type>
auto CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::controlDerivativeIntermediateNumDiff() -> control_vector_t
{
    throw std::runtime_error("controlDerivativeIntermediateNumDiff not implemented for manifold case.");
}

}  // namespace optcon
}  // namespace ct
