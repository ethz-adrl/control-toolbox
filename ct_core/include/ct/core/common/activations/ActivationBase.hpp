/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

namespace tpl {

/*!
 * Base class for all sort of (scalar) activation functions, e.g. time activations. Activations can be used to
 * periodically scale costs, constraints, or other, arbitrary functions. All activations are scalars.
 */
template <typename SCALAR>
class ActivationBase
{
public:
    //! constructor
    ActivationBase() = default;
    //! destructor
    virtual ~ActivationBase() = default;
    //! load activations from file
    virtual void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false)
    {
        throw std::runtime_error("No loadConfigFile method implemented for selected activation term");
    }

    //! return if term is active
    virtual bool isActive(const SCALAR s) { return true; }
    //! compute activation multiplier based on scalar input
    virtual SCALAR computeActivation(const SCALAR s) { return (SCALAR)1.0; }
    //! first order derivative of this activation
    virtual SCALAR firstOrderDerivative(const SCALAR s)
    {
        throw std::runtime_error("First order derivative not implemented for selected activation term");
    }
    //! second order derivative of this activation
    virtual SCALAR secondOrderDerivative(const SCALAR s)
    {
        throw std::runtime_error("Second order derivative not implemented for selected activation term");
    }
    //! print to console
    virtual void printInfo() {}
};
}  // namespace tpl

typedef tpl::ActivationBase<double> ActivationBase;
}  // namespace core
}  // namespace ct
