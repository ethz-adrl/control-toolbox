
#pragma once

#include "../activations/ActivationBase.hpp"
#include <ct/core/internal/traits/TraitSelectorSpecs.h>

namespace ct {
namespace core {
namespace tpl {

template <typename SCALAR, typename TRAIT = typename ct::core::tpl::TraitSelector<SCALAR>::Trait>
class SingleActivation : public ActivationBase<SCALAR>
{
public:
    SingleActivation() {}
    SingleActivation(const SCALAR t_on, const SCALAR t_off) : t_on_(t_on), t_off_(t_off) {}
    virtual ~SingleActivation() {}
    SingleActivation(const SingleActivation& arg) : t_on_(arg.t_on_), t_off_(arg.t_off_) {}
    virtual void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false) override
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(filename, pt);
        t_on_ = pt.get<SCALAR>(termName + ".t_on");
        t_off_ = pt.get<SCALAR>(termName + ".t_off");
    }

    //! this activation is active in a strict time window
    virtual bool isActive(const SCALAR t) override { return (t >= t_on_ && t < t_off_); }
    //! within this time-window, the activation is saturated
    virtual SCALAR computeActivation(const SCALAR t) override { return SCALAR(1.0); }
    virtual void printInfo() override
    {
        std::cout << "Cost Function active between values: " << t_on_ << "s and: " << t_off_ << "s" << std::endl;
    }

private:
    //! time when to switch on
    SCALAR t_on_;
    //! time when to switch off
    SCALAR t_off_;
};

}  // namespace tpl

typedef tpl::SingleActivation<double> SingleActivation;
}  // namespace core
}  // namespace ct
