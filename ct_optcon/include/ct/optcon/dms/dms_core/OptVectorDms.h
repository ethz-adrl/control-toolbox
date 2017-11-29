/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once


#include <Eigen/Core>
#include <map>

#include "TimeGrid.h"

#include <ct/optcon/dms/dms_core/DmsDimensions.h>
#include <ct/optcon/dms/dms_core/spline/SplinerBase.h>
#include <ct/optcon/dms/dms_core/spline/ZeroOrderHold/ZeroOrderHoldSpliner.h>
#include <ct/optcon/dms/dms_core/spline/Linear/LinearSpliner.h>
#include <ct/optcon/nlp/OptVector.h>
#include <ct/optcon/dms/dms_core/DmsSettings.h>


namespace ct {
namespace optcon {


/**
 * @ingroup    DMS
 *
 * @brief      This class is a wrapper around the NLP Optvector. It wraps the
 *             Vectors from the NLP solvers into state, control and time
 *             trajectories
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class OptVectorDms : public tpl::OptVector<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR> DIMENSIONS;
    typedef tpl::OptVector<SCALAR> Base;

    typedef typename DIMENSIONS::state_vector_t state_vector_t;
    typedef typename DIMENSIONS::control_vector_t control_vector_t;
    typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
    typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
    typedef typename DIMENSIONS::control_matrix_t control_matrix_t;
    typedef typename DIMENSIONS::time_array_t time_array_t;

    /**
	 * @brief      Custom constructor
	 *
	 * @param[in]  n         The number of optimization variables
	 * @param[in]  settings  The dms settings
	 */
    OptVectorDms(size_t n, const DmsSettings& settings);

    OptVectorDms(const OptVectorDms& arg) = delete;

    /**
	 * @brief      Destructor
	 */
    virtual ~OptVectorDms() {}
    /**
	 * @brief      Returns the optimized state for a specific shot
	 *
	 * @param[in]  pairNum  The shot number
	 *
	 * @return     The optimized state
	 */
    state_vector_t getOptimizedState(const size_t pairNum) const;

    /**
	 * @brief      Returns the optimized control input for a specific shot
	 *
	 * @param[in]  pairNum  The shot number
	 *
	 * @return     The optimized control input
	 */
    control_vector_t getOptimizedControl(const size_t pairNum) const;

    /**
	 * @brief      Return the optimized time segment for a specific shot
	 *
	 * @param[in]  pairNum  The pair number
	 *
	 * @return     The optimized time segment.
	 */
    SCALAR getOptimizedTimeSegment(const size_t pairNum) const;

    /**
	 * @brief      Returns the optimized state for all shots
	 *
	 * @return     The optimized states.
	 */
    const state_vector_array_t& getOptimizedStates();

    /**
	 * @brief      Returns the optimized control inputs for all shots
	 *
	 * @return     The optimized control inputs.
	 */
    const control_vector_array_t& getOptimizedInputs();

    /**
	 * @brief      Returns the optimized time segments for all shots
	 *
	 * @return     The optimized time segments.
	 */
    const Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>& getOptimizedTimeSegments();

    /**
	 * @brief      Returns the starting index for the state at shot pairNum
	 *             inside the optimization vector
	 *
	 * @param[in]  pairNum  The shot number
	 *
	 * @return     The state index.
	 */
    size_t getStateIndex(const size_t pairNum) const;

    /**
	 * @brief      Returns the starting index for the control input at shot pairNum
	 *             inside the optimization vector
	 *
	 * @param[in]  pairNum  The shot number
	 *
	 * @return     The state index.
	 */
    size_t getControlIndex(const size_t pairNum) const;

    /**
	 * @brief      Returns the starting index for the time segment input at shot pairNum
	 *             inside the optimization vector
	 *
	 * @param[in]  pairNum  The shot number
	 *
	 * @return     The state index.
	 */
    size_t getTimeSegmentIndex(const size_t pairNum) const;


    /**
	 * @brief      Sets an initial guess for the optimal solution. The optimal
	 *             solution is set as a linear interpolation between inital
	 *             state x0 and final state xf. The initial guess for the
	 *             control input is assumed to be constant and equal to u0
	 *
	 * @param[in]  x0    The initial state
	 * @param[in]  x_f   The final state
	 * @param[in]  u0    The control input
	 */
    void setInitGuess(const state_vector_t& x0, const state_vector_t& x_f, const control_vector_t& u0);

    /**
	 * @brief      Sets an initial guess for the optimal solution.
	 *
	 * @param[in]  x_init  Initial guess for the state trajectory
	 * @param[in]  u_init  Initial guess for the control trajectory
	 */
    void setInitGuess(const state_vector_array_t& x_init, const control_vector_array_t& u_init);

    /**
	 * @brief      Updates the initial state
	 *
	 * @param[in]  x0    The new initial state
	 */
    void changeInitialState(const state_vector_t& x0);

    /**
	 * @brief      Updates the final state
	 *
	 * @param[in]  xF    The new final state
	 */
    void changeDesiredState(const state_vector_t& xF);

    /**
	 * @brief      Sets bounds on the time segment variables
	 */
    void setLowerTimeSegmentBounds();

    /**
	 * @brief      Returns the number of pairs 
	 *
	 * @return     Number of pairs
	 */
    size_t numPairs() { return numPairs_; }
    /**
	 * @brief      Prints out the solution trajectories
	 */
    void printoutSolution();

private:
    DmsSettings settings_;

    size_t numPairs_;
    /* maps the number of a "pair" to the index in w where ... */
    std::map<size_t, size_t> pairNumToStateIdx_;   /* ... its state starts */
    std::map<size_t, size_t> pairNumToControlIdx_; /* ... its control starts */
    std::map<size_t, size_t>
        shotNumToShotDurationIdx_; /* its shot time starts is in w (last element doesn't have one) */

    state_vector_array_t stateSolution_;
    control_vector_array_t inputSolution_;
    Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> optimizedTimeSegments_;
};

}  // namespace optcon
}  // namespace ct

#include "implementation/OptVectorDms-impl.h"
