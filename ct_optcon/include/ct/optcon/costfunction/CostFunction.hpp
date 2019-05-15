/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <vector>

namespace ct {
namespace optcon {

/** \defgroup CostFunction CostFunction
 *
 * \brief Cost Function Module used in Optimal Control
 */

/**
 * \ingroup CostFunction
 *+
 * \brief A base function for cost functions. All cost functions should derive from this.
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class CostFunction
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;

    /**
	 * \brief Default constructor
	 *
	 * Default constructor, sets state, control and time to zero
	 */
    CostFunction();

    /**
	 * \brief Destructor
	 *
	 * Destructor
	 */
    virtual ~CostFunction();

    /**
	 * \brief Copy constructor
	 * @param arg other cost function
	 */
    CostFunction(const CostFunction& arg);

    /**
	 * Clones the cost function.
	 * @return Base pointer to the clone
	 */
    virtual CostFunction<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const = 0;

    /**
	 * Set the current state, control and time of the cost function. In this function, the user can add pre-computations
	 * shared between different calls to the derivative functions.
	 * @param x state vector
	 * @param u control vector
	 * @param t time
	 */
    virtual void setCurrentStateAndControl(const state_vector_t& x,
        const control_vector_t& u,
        const SCALAR& t = SCALAR(0.0));

    /**
	 * \brief sets current state, control and time
	 *
	 * Get the current state, control and time of the cost function. In this function, the user can add pre-computations
	 * shared between different calls to the derivative functions.
	 * @param x state vector
	 * @param u control vector
	 * @param t time
	 */
    virtual void getCurrentStateAndControl(Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
        Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
        SCALAR& t) const;

    /**
	 * \brief evaluate intermediate costs
	 *
	 * Evaluates the running/intermediate cost function for the control, state and time set in setCurrentStateAndControl()
	 * @return costs
	 */
    virtual SCALAR evaluateIntermediate() = 0;

    /**
	 * \brief evaluate terminal costs
	 *
	 * Evaluates the terminal cost for a given state and control set in setCurrentStateAndControl(). This usually ignores time.
	 * @return costs
	 */
    virtual SCALAR evaluateTerminal() = 0;

    virtual void shiftTime(const SCALAR t);


protected:
    state_vector_t x_;   /** state vector */
    control_vector_t u_; /** control vector */
    SCALAR t_;           /** time */

    SCALAR t_shift_;
};

}  // namespace optcon
}  // namespace ct
