/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#pragma once

#include <boost/algorithm/string.hpp>

#include <ct/core/common/activations/Activations.h>

namespace ct {
namespace optcon {

/*!
 * \ingroup CostFunction
 *
 * \brief An interface for a term, supporting both analytical and auto-diff terms
 *
 * Derive from this term to implement your own term. You only have to implement
 * evaluateCppadCg() if you want to use auto-diff. Otherwise, you need to implement
 * evaluate() as well as the derivatives. In case you want to go for the most general
 * implementation, you can implement a local, templated evalLocal() method in derived terms,
 * which gets called by evaluate() and evaluateCppadCg(), see e.g. TermLinear.
 *
 * An example for an implementation of a custom term is given in \ref TermQuadratic.hpp
 **/
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL = double, typename SCALAR = SCALAR_EVAL>
class TermBase
{
protected:
    //! a name identifier for this term
    std::string name_;
    //! time activations for this term
    std::shared_ptr<ct::core::tpl::ActivationBase<SCALAR_EVAL>> c_i_;

public:
    typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM> control_state_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM> state_matrix_double_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM> control_matrix_double_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM> control_state_matrix_double_t;

    /**
	 * \brief Default constructor
	 * @param name Name of the term
	 */
    TermBase(std::string name = "Unnamed");

    /**
	 * \brief Copy Cunstructor
	 * @param arg The term to copy
	 */
    TermBase(const TermBase& arg);

    /**
	 * \brief Deep-copy term
	 * @return
	 */
    virtual TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>* clone() const = 0;

    /**
	 * \brief Destructor
	 */
    virtual ~TermBase();

    /**
	 * @brief      Evaluates the term at x, u, t
	 *
	 * @param[in]  x     The current state
	 * @param[in]  u     The current control
	 * @param[in]  t     The current time
	 *
	 * @return     The evaluatated cost term
	 */
    virtual SCALAR evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
        const Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
        const SCALAR& t) = 0;

#ifdef CPPADCG
    /**
	 * @brief      The evaluate method used for jit compilation in CostfunctionAD
	 *
	 * @param[in]  x     The state vector
	 * @param[in]  u     The control vector
	 * @param[in]  t     The time
	 *
	 * @return     The evaluated cost
	 */
    virtual ct::core::ADCGScalar evaluateCppadCg(const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
        const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t);
#endif

    /**
	 * @brief      Gets called by the analytical costfunction. Adds time
	 *             dependent activations on top of the term
	 *
	 * @param[in]  x     The current state
	 * @param[in]  u     The current control
	 * @param[in]  t     The current time
	 *
	 * @return     The evaluatated cost term
	 */
    SCALAR_EVAL eval(const Eigen::Matrix<SCALAR_EVAL, STATE_DIM, 1>& x,
        const Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, 1>& u,
        const SCALAR_EVAL& t);

    /**
	 * \brief Returns if term is non-zero at a specific time
	 * By default, all terms are evaluated at all times. However, if a term is not active at a certain time, you can overload this
	 * function to spare evaluations of the term and its derivatives
	 * @param t time
	 * @return true if term is active at t
	 */
    virtual bool isActiveAtTime(SCALAR_EVAL t);

    //! compute time activation
    SCALAR_EVAL computeActivation(SCALAR_EVAL t);

    //! compute derivative of this cost term w.r.t. the state
    virtual core::StateVector<STATE_DIM, SCALAR_EVAL> stateDerivative(
        const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t);

    //! compute second order derivative of this cost term w.r.t. the state
    virtual state_matrix_t stateSecondDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t);

    //! compute derivative of this cost term w.r.t. the control input
    virtual core::ControlVector<CONTROL_DIM, SCALAR_EVAL> controlDerivative(
        const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t);

    //! compute second order derivative of this cost term w.r.t. the control input
    virtual control_matrix_t controlSecondDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t);

    //! compute the cross-term derivative (state-control) of this cost function term
    virtual control_state_matrix_t stateControlDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t);

    //! load this term from a configuration file
    virtual void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false);

    //! set the time activation functions for this term
    void setTimeActivation(std::shared_ptr<ct::core::tpl::ActivationBase<SCALAR_EVAL>> c_i, bool verbose = false);

    //! load the time activation functions for this term from file
    void loadTimeActivation(const std::string& filename, const std::string& termName, bool verbose = false);

    /**
	 * \brief Returns the name of the term
	 * @param termName name of the term
	 */
    const std::string& getName() const;

    /**
	 * \brief Sets the name of the term
	 * @param termName
	 */
    void setName(const std::string& termName);

    //! updates the reference state for this term
    virtual void updateReferenceState(const Eigen::Matrix<SCALAR_EVAL, STATE_DIM, 1>& newRefState);

    //! updates the reference control for this term
    virtual void updateReferenceControl(const Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, 1>& newRefControl);

    //! retrieve this term's current reference state
    virtual Eigen::Matrix<SCALAR_EVAL, STATE_DIM, 1> getReferenceState() const;
};

}  // namespace optcon
}  // namespace ct
