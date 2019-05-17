/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "ActivationBase.hpp"
#include <ct/core/internal/traits/TraitSelectorSpecs.h>

namespace ct {
namespace core {

namespace tpl {

/*!
 * A scalar barrier term of the form
 *  \f[
 *  f(x) = exp(\alpha(x-ub)) + exp(\alpha(lb-x))
 * \f]
 * with
 * \f[
 *  \alpha > 0
 * \f]
 * where *ub* denotes the upper bound, *lb* denotes the lower bound, and the parameter *alpha* controls the
 * steepness of the flanks. The analytical derivatives for this term are straight-forward.
 */
template <typename SCALAR, typename TRAIT = typename ct::core::tpl::TraitSelector<SCALAR>::Trait>
class BarrierActivation : public ActivationBase<SCALAR>
{
public:
    //! trivial constructor, which deactivates upper and lower bounds.
    BarrierActivation() : ub_active_(false), lb_active_(false), ub_((SCALAR)0.0), lb_((SCALAR)0.0), alpha_((SCALAR)0.0)
    {
    }
    /*!
     * \brief constructor with input arguments for upper and lower bound
     * @param ub the upper bound barrier
     * @param lb the lower bound barrier
     * @param alpha scaling factor (steepness multiplier)
     */
    BarrierActivation(const SCALAR& ub, const SCALAR& lb, const SCALAR& alpha = SCALAR(1.0))
        : ub_active_(true), lb_active_(true), ub_(ub), lb_(lb), alpha_(alpha)
    {
    }
    //! destructor
    virtual ~BarrierActivation() {}
    //! load activations from file
    virtual void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false)
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(filename, pt);

        // load the steepness multiplier
        alpha_ = pt.get<SCALAR>(termName + ".alpha");

        try
        {
            // if there is no or a wrongly specified upper bound
            ub_ = pt.get<SCALAR>(termName + ".upper_bound");
        } catch (std::exception& e)
        {
            // we set the upper bound inactive
            std::cout << e.what() << std::endl;
            ub_active_ = false;
        }

        try
        {
            // if there is no or a wrongly specified lower bound
            lb_ = pt.get<SCALAR>(termName + ".lower_bound");
        } catch (std::exception& e)
        {
            // we set the lower bound inactive
            std::cout << e.what() << std::endl;
            lb_active_ = false;
        }

        // do some checks
        if ((lb_active_ && ub_active_) && (lb_ > ub_))
        {
            throw std::runtime_error("BarrierActivation: lower bound cannot be greater than upper bound.");
        }
        if (alpha_ < 0)
        {
            throw std::runtime_error("BarrierActivation: alpha must be >= 0");
        }
    }


    /*!
     * \brief activate the lower bound while deactivating the upper bound.
     * @param lb new numeric value for the lower bound threshold
     * @param alpha new steepness multiplier
     */
    void setLowerBoundOnly(const SCALAR& lb, const SCALAR& alpha = (SCALAR)1.0)
    {
        lb_active_ = true;
        ub_active_ = false;
        lb_ = lb;
        alpha_ = alpha;
    }

    /*!
     * \brief activate the upper bound while deactivating the lower bound
     * @param ub new numeric value for the upper bound threshold
     * @param alpha new steepness multiplier
     */
    void setUpperBoundOnly(const SCALAR& ub, const SCALAR& alpha = (SCALAR)1.0)
    {
        lb_active_ = false;
        ub_active_ = true;
        ub_ = ub;
        alpha_ = alpha;
    }

    //! compute activation multiplier based on scalar input
    virtual SCALAR computeActivation(const SCALAR x) override
    {
        return (SCALAR)ub_active_ * TRAIT::exp(alpha_ * (x - ub_)) +
               (SCALAR)lb_active_ * TRAIT::exp(alpha_ * (lb_ - x));
    }
    //! first order derivative of this activation
    virtual SCALAR firstOrderDerivative(const SCALAR x) override
    {
        return (SCALAR)ub_active_ * alpha_ * TRAIT::exp(alpha_ * (x - ub_)) -
               (SCALAR)lb_active_ * alpha_ * TRAIT::exp(alpha_ * (lb_ - x));
    }
    //! second order derivative of this activation
    virtual SCALAR secondOrderDerivative(const SCALAR x)
    {
        return (SCALAR)ub_active_ * alpha_ * alpha_ * TRAIT::exp(alpha_ * (x - ub_)) +
               (SCALAR)lb_active_ * alpha_ * alpha_ * TRAIT::exp(alpha_ * (lb_ - x));
    }
    //! print to console
    virtual void printInfo()
    {
        if (ub_active_)
        {
            std::cout << "Barrier Activation with upper bound " << ub_ << ", steepness-multiplier " << alpha_
                      << std::endl;
        }
        if (lb_active_)
        {
            std::cout << "Barrier Activation with lower bound " << lb_ << ", steepness-multiplier " << alpha_
                      << std::endl;
        }
    }

protected:
    //! boolean indicating if upper bound is activated
    bool ub_active_;
    //! boolean indicating of lower bound is activated
    bool lb_active_;
    //! the upper bound
    SCALAR ub_;
    //! the upper bound
    SCALAR lb_;
    //! steepness-multiplier (controls how 'aggressive' the boundary is)
    SCALAR alpha_;
};
}  // namespace tpl

typedef tpl::BarrierActivation<double, ct::core::internal::DoubleTrait> BarrierActivation;
}  // namespace core
}  // namespace ct
