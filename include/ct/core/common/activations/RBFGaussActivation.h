
#pragma once

#include <cmath>
#include <math.h>

#include "../activations/ActivationBase.hpp"
#include <ct/core/internal/traits/TraitSelectorSpecs.h>

namespace ct {
namespace core {
namespace tpl {

template <typename SCALAR, typename TRAIT = typename ct::core::tpl::TraitSelector<SCALAR>::Trait>
class RBFGaussActivation : public ActivationBase<SCALAR>
{
public:
    RBFGaussActivation() {}
    RBFGaussActivation(const SCALAR mu, const SCALAR sigma) : mu_(mu), sigma_(sigma)
    {
        sigma2inv_ = SCALAR(1.0) / (SCALAR(2.0) * sigma_ * sigma_);
        factor_ = SCALAR(1.0) / sqrt(SCALAR(2.0) * M_PI * sigma_ * sigma_);
    }
    virtual ~RBFGaussActivation() {}
    RBFGaussActivation(const RBFGaussActivation& arg)
        : mu_(arg.mu_), sigma_(arg.sigma_), sigma2inv_(arg.sigma2inv_), factor_(arg.factor_)
    {
    }

    virtual void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false) override
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(filename, pt);
        mu_ = pt.get<SCALAR>(termName + ".mu");
        sigma_ = pt.get<SCALAR>(termName + ".sigma");
        // factors used for efficient computeActivation calculation
        sigma2inv_ = -SCALAR(1.0) / (SCALAR(2.0) * sigma_ * sigma_);
        factor_ = SCALAR(1.0) / sqrt(SCALAR(2.0) * M_PI * sigma_ * sigma_);
    }

    virtual bool isActive(const SCALAR t) override { return true; }
    virtual SCALAR computeActivation(const SCALAR t) override
    {
        return factor_ * exp((t - mu_) * (t - mu_) * sigma2inv_);
    }

    virtual void printInfo() override
    {
        std::cout << "RBF costfunction time activation" << std::endl;
        std::cout << "mu: " << mu_ << "s \t sigma: " << sigma_ << "s" << std::endl;
    }

private:
    SCALAR mu_;
    SCALAR sigma_;
    SCALAR sigma2inv_;
    SCALAR factor_;
};
}  //namespace tpl

typedef tpl::RBFGaussActivation<double> RBFGaussActivation;
}  //namespact core
}  //namespace ct
