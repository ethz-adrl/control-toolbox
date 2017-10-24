/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Lincensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <boost/concept_check.hpp>
#include <memory>

#include <ct/core/types/Time.h>

namespace ct {
namespace core {

//! A general SISO controller interface
/*!
 * A standard interface defining a single-input single-output controller, such as a PIDController.
 * Inherit from this class to implement your custom SISO controller.
 */
class SISOControllerBase
{
public:
    //! Default constructor
    SISOControllerBase(){};

    //! Copy constructor
    SISOControllerBase(const SISOControllerBase& arg) {}
    //! Destructor
    virtual ~SISOControllerBase(){};

    //! Deep cloning destructor
    /*!
	 * Needs to be implemented by derived class.
	 * @return pointer to cloned instance
	 */
    virtual SISOControllerBase* clone() const = 0;

    //! Computes the control action
    /*!
	 * Takes the current state and time and computes the corresponding output.
	 * Needs to be implemented by any derived class.
	 * @param state current state
	 * @param t current time
	 * @return resulting control action
	 */
    virtual double computeControl(const double& state, const core::Time& t) = 0;

protected:
};

}  // namespace core
}  // namespace ct
