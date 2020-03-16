#pragma once

#include "../activations/ActivationBase.hpp"
#include <ct/core/internal/traits/TraitSelectorSpecs.h>

namespace ct {
namespace core {
namespace tpl {

template <typename SCALAR, typename TRAIT = typename ct::core::tpl::TraitSelector<SCALAR>::Trait>
class LinearActivation : public ActivationBase<SCALAR>
{
public:
    LinearActivation() = default;
    LinearActivation(const SCALAR t_on, const SCALAR t_off, const SCALAR slope, const SCALAR startVal)
        : t_on_(t_on), t_off_(t_off), slope_(slope), startVal_(startVal)
    {
    }
    virtual ~LinearActivation() = default;
    LinearActivation(const LinearActivation& arg)
        : t_on_(arg.t_on_), t_off_(arg.t_off_), slope_(arg.slope_), startVal_(arg.startVal_)
    {
    }
    virtual void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false) override
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(filename, pt);
        t_on_ = pt.get<SCALAR>(termName + ".t_on");
        t_off_ = pt.get<SCALAR>(termName + ".t_off");
        slope_ = pt.get<SCALAR>(termName + ".slope");
        startVal_ = pt.get<SCALAR>(termName + ".startVal");
    }

    //! this activation is active in a strict time window
    virtual bool isActive(const SCALAR t) override { return (t >= t_on_ && t < t_off_); }
    //! within this time-window, the activation is saturated
    virtual SCALAR computeActivation(const SCALAR t) override { return startVal_ + slope_ * (t - t_on_); }
    virtual void printInfo() override
    {
        std::cout << "Cost function with linear activation active from " << t_on_ << " s with value " << startVal_
                  << ", increasing by " << slope_ << " per second and ending at " << t_off_ << " s." << std::endl;
    }

private:
    SCALAR t_on_;      //! time when to switch on
    SCALAR t_off_;     //! time when to switch off
    SCALAR slope_;     //! change of activation per unit of time
    SCALAR startVal_;  //! value at t_on
};

}  // namespace tpl

typedef tpl::LinearActivation<double> LinearActivation;
}  // namespace core
}  // namespace ct
