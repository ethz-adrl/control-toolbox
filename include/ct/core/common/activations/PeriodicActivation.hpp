
#pragma once

#include <cmath>

#include "../activations/ActivationBase.hpp"
#include <ct/core/internal/traits/TraitSelectorSpecs.h>

namespace ct {
namespace core {
namespace tpl {

template <typename SCALAR, typename TRAIT = typename ct::core::tpl::TraitSelector<SCALAR>::Trait>
class PeriodicActivation : public ActivationBase<SCALAR>
{
public:
    PeriodicActivation() {}
    PeriodicActivation(const SCALAR active_percentage,
        const SCALAR period,
        const SCALAR activation_offset,
        const SCALAR period_offset)
        : active_percentage_(active_percentage),
          period_(period),
          activation_offset_(activation_offset),
          period_offset_(period_offset)
    {
    }
    virtual ~PeriodicActivation() {}
    PeriodicActivation(const PeriodicActivation& arg)
        : active_percentage_(arg.active_percentage_),
          period_(arg.period_),
          activation_offset_(arg.activation_offset_),
          period_offset_(arg.period_offset_),
          t_end_(arg.t_end_)
    {
    }

    virtual void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false) override
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(filename, pt);

        active_percentage_ = pt.get<SCALAR>(termName + ".active_percentage");
        period_ = pt.get<SCALAR>(termName + ".period");
        activation_offset_ = pt.get<SCALAR>(termName + ".activation_offset");
        period_offset_ = pt.get<SCALAR>(termName + ".period_offset");
        t_end_ = pt.get<SCALAR>(termName + ".t_end");

        if (activation_offset_ + active_percentage_ * period_ > period_)
        {
            throw std::runtime_error(
                "Activation offset plus active period percentage exceed period time. Adjust settings");
        }

        if (verbose)
            printInfo();
    }

    // to verify
    virtual bool isActive(const SCALAR t) override { return isActiveSpecialized(t); }
    template <typename S = SCALAR>
    typename std::enable_if<std::is_same<S, double>::value, bool>::type isActiveSpecialized(const SCALAR t)
    {
        bool active = false;
        if (t >= period_offset_ && t < t_end_)
        {
            SCALAR t0 = t - period_offset_;
            SCALAR t0norm = fmod(t0, period_);
            if (t0norm >= activation_offset_ && t0norm < (activation_offset_ + active_percentage_ * period_))
                active = true;
        }
        return active;
    }

    template <typename S = SCALAR>
    typename std::enable_if<!std::is_same<S, double>::value, bool>::type isActiveSpecialized(const SCALAR t)
    {
        return true;
    }

    virtual SCALAR computeActivation(const SCALAR t) override { return SCALAR(1.0); }
    virtual void printInfo() override
    {
        std::cout << "Cost function active at periodic times: " << std::endl;
        std::cout << "Period: " << period_ << "\nOffset after period start:  " << activation_offset_ << "s"
                  << std::endl;
        std::cout << "Offset between t0: " << period_offset_ << "s" << std::endl;
        std::cout << "Active for " << 100 * active_percentage_ << "% of the period" << std::endl;
    }

private:
    SCALAR
    active_percentage_;  // how much of the cycle is the time active TODO: misleading name. should be called fraction
    SCALAR period_;      // what is the period
    SCALAR activation_offset_;  // how much is the activation offset WITHIN the period
    SCALAR period_offset_;      // how much is the period offset to t=0?
    SCALAR t_end_;
};
}  // namespace tpl

typedef tpl::PeriodicActivation<double> PeriodicActivation;
}  // namespace core
}  // namespace ct
