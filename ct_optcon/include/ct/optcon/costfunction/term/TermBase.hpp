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
template <typename MANIFOLD, size_t CONTROL_DIM>
class TermBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr size_t STATE_DIM = MANIFOLD::TangentDim;
    using SCALAR = typename MANIFOLD::Scalar;

    //!< define input/output scalar type of resulting evaluations
    using SCALAR_EVAL = typename ct::core::get_out_type<MANIFOLD>::type;
    using EVAL_MANIFOLD = typename MANIFOLD::template RedefineScalar<SCALAR_EVAL>;

    using state_matrix_t = ct::core::StateMatrix<STATE_DIM, SCALAR_EVAL>;
    using control_matrix_t = ct::core::ControlMatrix<SCALAR_EVAL>;
    using control_state_matrix_t = ct::core::ControlStateMatrix<STATE_DIM, SCALAR_EVAL>;

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
    virtual TermBase<MANIFOLD, CONTROL_DIM>* clone() const = 0;


    /**
	 * @brief      Evaluates the term at x, u, t
	 *
	 * @param[in]  x     The current state
	 * @param[in]  u     The current control
	 * @param[in]  t     The current time
	 *
	 * @return     The evaluatated cost term, use SCALAR for compatibility with both autodiff and analytic evaluation
	 */
    virtual SCALAR evaluate(const MANIFOLD& x,
        const ct::core::ControlVector<SCALAR>& u,
        const SCALAR& t) = 0;

    /**
	 * \brief Returns if term is non-zero at a specific time
	 * By default, all terms are evaluated at all times. However, if a term is not active at a certain time, you can overload this
	 * function to spare evaluations of the term and its derivatives
	 * @param t time
	 * @return true if term is active at t
	 */
    virtual bool isActiveAtTime(SCALAR_EVAL t);

    //! compute time activation
    SCALAR_EVAL computeActivation(const SCALAR_EVAL t);

    //! compute derivative of this cost term w.r.t. the state
    virtual core::StateVector<STATE_DIM, SCALAR_EVAL> stateDerivative(const EVAL_MANIFOLD& x,
        const core::ControlVector<SCALAR_EVAL>& u,
        const SCALAR_EVAL& t);

    //! compute second order derivative of this cost term w.r.t. the state
    virtual state_matrix_t stateSecondDerivative(const EVAL_MANIFOLD& x,
        const core::ControlVector<SCALAR_EVAL>& u,
        const SCALAR_EVAL& t);

    //! compute derivative of this cost term w.r.t. the control input
    virtual core::ControlVector<SCALAR_EVAL> controlDerivative(const EVAL_MANIFOLD& x,
        const core::ControlVector<SCALAR_EVAL>& u,
        const SCALAR_EVAL& t);

    //! compute second order derivative of this cost term w.r.t. the control input
    virtual control_matrix_t controlSecondDerivative(const EVAL_MANIFOLD& x,
        const core::ControlVector<SCALAR_EVAL>& u,
        const SCALAR_EVAL& t);

    //! compute the cross-term derivative (state-control) of this cost function term
    virtual control_state_matrix_t stateControlDerivative(const EVAL_MANIFOLD& x,
        const core::ControlVector<SCALAR_EVAL>& u,
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
    virtual void updateReferenceState(const EVAL_MANIFOLD& newRefState);

    //! updates the reference control for this term
    virtual void updateReferenceControl(const ct::core::ControlVector<SCALAR_EVAL>& newRefControl);

    //! retrieve this term's current reference state
    virtual EVAL_MANIFOLD getReferenceState() const;

protected:
    //! a name identifier for this term
    std::string name_;
    //! time activations for this term
    std::shared_ptr<ct::core::tpl::ActivationBase<SCALAR_EVAL>> c_i_;
};

}  // namespace optcon
}  // namespace ct
