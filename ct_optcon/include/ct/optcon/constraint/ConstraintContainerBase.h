/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

/** \defgroup Constraint Constraint
 *
 * \brief Constraint Module used in Optimal Control
 */

/**
 * @ingroup    Constraint
 * + { list_item_description }
 *
 * @brief      The ConstraintBase Class is the base class for defining the
 *             non-linear optimization constraints.
 *
 * @tparam     STATE_DIM  Dimension of the state vector
 * @tparam     CONTROL_DIM  Dimension of the control input vector
 *
 *             An example for usage is given in the unit test @ref
 *             ConstraintTest.h
 */

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ConstraintContainerBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef core::ControlVector<CONTROL_DIM, SCALAR> input_vector_t;

    typedef ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>* ConstraintBase_Raw_Ptr_t;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;

    /**
	 * \brief Default constructor
	 *
	 * Default constructor, sets state, control and time to zero
	 */
    ConstraintContainerBase();

    /**
	 * \brief copy constructor
	 *
	 * Copy constructor
	 */
    ConstraintContainerBase(const ConstraintContainerBase& arg);

    /**
	 * \brief Destructor
	 *
	 * Destructor
	 */
    virtual ~ConstraintContainerBase();

    /**
	 * Clones the constraint.
	 * @return Base pointer to the clone
	 */
    virtual ConstraintBase_Raw_Ptr_t clone() const = 0;

    /**
	 * This methods updates the current time, state, and input in the class. It also call the user defined update() method,
	 * which can be used to make the implementation more efficient by executing shared calculations directly at the time of
	 * updating time, state and control.
	 *
	 * @param[in] t current time
	 * @param[in] x state vector
	 * @param[in] x input vector
	 */
    virtual void setCurrentStateAndControl(const state_vector_t& x,
        const input_vector_t& u,
        const SCALAR t = SCALAR(0.0));

    /**
	 * @brief      Evaluates the intermediate constraints
	 *
	 * @return     The evaluation of the intermediate constraints
	 */
    virtual VectorXs evaluateIntermediate() = 0;

    /**
	 * @brief      Evaluates the terminal constraints
	 *
	 * @return     The evaluation of the terminal constraints
	 */
    virtual VectorXs evaluateTerminal() = 0;

    /**
	 * @brief      Retrieves the number of intermediate constraints
	 *
	 * @return     The number of intermediate constraints
	 */
    virtual size_t getIntermediateConstraintsCount() = 0;

    /**
	 * @brief      Retrieves the number of final constraints
	 *
	 * @return     The number of final constraints
	 */
    virtual size_t getTerminalConstraintsCount() = 0;

    /**
	 * @brief      Retrieves the total number of constraints
	 *
	 * @return     The total number of constraints
	 */
    size_t getConstraintsCount();
    /**
	 * @brief      Retrieves the lower constraint bound on the intermediate
	 *             constraints
	 *
	 * @return     The lower bound on the intermediate constraints
	 */
    VectorXs getLowerBoundsIntermediate() const;

    /**
	 * @brief      Retrieves the lower constraint bound on the terminal
	 *             constraints
	 *
	 * @return     The lower bound on the terminal constraints
	 */
    VectorXs getLowerBoundsTerminal() const;

    /**
	 * @brief      Retrieves the upper constraint bound on the intermediate
	 *             constraints
	 *
	 * @return     The upper bound on the intermediate constraints
	 */
    VectorXs getUpperBoundsIntermediate() const;

    /**
	 * @brief      Retrieves the upper constraint bound on the terminal
	 *             constraints
	 *
	 * @return     The upper bound on the terminal constraints
	 */
    VectorXs getUpperBoundsTerminal() const;

    /**
	 * @brief      Retrieves the violation of the upper constraint bound on the intermediate constraints
	 *
	 * @return     The upper bound violation on intermediate constraints
	 */
    VectorXs getUpperBoundsViolationIntermediate();

    /**
	 * @brief      Retrieves the violation of the lower constraint bound on the intermediate constraints
	 *
	 * @return     The lower bound violation on intermediate constraints
	 */
    VectorXs getLowerBoundsViolationIntermediate();

    /**
	 * @brief      Retrieves the violation of the upper constraint bound on the terminal constraints
	 *
	 * @return     The upper bound violation on terminal constraints
	 */
    VectorXs getUpperBoundsViolationTerminal();

    /**
	 * @brief      Retrieves the violation of the lower constraint bound on the terminal constraints
	 *
	 * @return     The lower bound violation on terminal constraints
	 */
    VectorXs getLowerBoundsViolationTerminal();

    /**
	 * @brief      Retrieves the total violation of the constraints bounds on the intermediate constraints
	 *
	 * @return     The total bound violation on intermediate constraints
	 */
    VectorXs getTotalBoundsViolationIntermediate();

    /**
	 * @brief      Retrieves the total violation of the constraints bounds on the terminal constraints
	 *
	 * @return     The total bound violation on terminal constraints
	 */
    VectorXs getTotalBoundsViolationTerminal();

protected:
    /**
	 * @brief      Gets called by the setCurrentStateAndControl method. Can be
	 *             used to update container properties
	 */
    virtual void update() = 0;

    state_vector_t x_; /** state vector */
    input_vector_t u_; /** control vector */
    SCALAR t_;         /** time */

    VectorXs lowerBoundsIntermediate_;
    VectorXs lowerBoundsTerminal_;
    VectorXs upperBoundsIntermediate_;
    VectorXs upperBoundsTerminal_;
};


}  // namespace optcon
}  // namespace ct
