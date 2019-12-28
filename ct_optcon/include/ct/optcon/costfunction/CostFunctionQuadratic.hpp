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
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class CostFunctionQuadratic : public CostFunction<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> control_state_matrix_t;

    typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;

    typedef CostFunction<STATE_DIM, CONTROL_DIM, SCALAR> BASE;

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
    virtual CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const = 0;

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
    virtual size_t addIntermediateTerm(std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>> term,
        bool verbose = false);

    /**
	 * \brief Adds a final term
	 * @param term final term
	 * @param verbose verbosity flag which enables printout
	 * @return
	 */
    virtual size_t addFinalTerm(std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>> term, bool verbose = false);


#ifdef CPPADCG
    virtual void addIntermediateADTerm(
        std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, ct::core::ADCGScalar>> term,
        bool verbose = false);

    virtual void addFinalADTerm(std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, ct::core::ADCGScalar>> term,
        bool verbose = false);
#endif

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
    virtual state_vector_t stateDerivativeIntermediate() = 0;

    /**
	 * Computes terminal-cost first-order derivative with respect to state
	 * @return derivative vector (Jacobian)
	 */
    virtual state_vector_t stateDerivativeTerminal() = 0;

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
    virtual void updateReferenceState(const state_vector_t& x_ref);

    //! update the reference state for final cost terms
    virtual void updateFinalState(const state_vector_t& x_final);

    //! update the reference control for intermediate cost terms
    virtual void updateReferenceControl(const control_vector_t& u_ref);

    //! compare the state derivative against numerical differentiation
    bool stateDerivativeIntermediateTest(bool verbose = false);

    //! compare the control derivative against numerical differentiation
    bool controlDerivativeIntermediateTest(bool verbose = false);

    std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>> getIntermediateTermById(const size_t id);

    std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>> getFinalTermById(const size_t id);

    std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>> getIntermediateTermByName(const std::string& name);

    std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>> getFinalTermByName(const std::string& name);

    //! initialize the cost function (e.g. to be used in CostFunctionAD)
    virtual void initialize();

protected:
    //! evaluate intermediate analytical cost terms
    SCALAR evaluateIntermediateBase();

    //! evaluate terminal analytical cost terms
    SCALAR evaluateTerminalBase();

    //! evaluate intermediate analytical state derivatives
    state_vector_t stateDerivativeIntermediateBase();

    //! evaluate terminal analytical state derivatives
    state_vector_t stateDerivativeTerminalBase();

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

    //! compute the state derivative by numerical differentiation (can be used for testing)
    state_vector_t stateDerivativeIntermediateNumDiff();

    //! compute the control derivative by numerical differentiation (can be used for testing)
    control_vector_t controlDerivativeIntermediateNumDiff();

    //! stepsize for numerical differentiation
    SCALAR eps_;

    //! use double sided derivatives in numerical differentiation
    bool doubleSidedDerivative_ = true;

    /** list of intermediate cost terms for which analytic derivatives are available */
    std::vector<std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>>> intermediateCostAnalytical_;

    /** list of final cost terms for which analytic derivatives are available */
    std::vector<std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>>> finalCostAnalytical_;
};


}  // namespace optcon
}  // namespace ct
