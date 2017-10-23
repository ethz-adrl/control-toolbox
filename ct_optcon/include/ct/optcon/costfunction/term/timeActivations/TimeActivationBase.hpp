
#pragma once

namespace ct {
namespace optcon {

namespace tpl {

template <typename SCALAR>
class TimeActivationBase
{
public:
    TimeActivationBase() {}
    virtual ~TimeActivationBase() {}
    virtual void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false)
    {
        return;
    }

    virtual bool isActiveAtTime(const SCALAR t) { return true; }
    // virtual

    virtual SCALAR computeActivation(const SCALAR t) { return SCALAR(1.0); }
    virtual void printInfo() { std::cout << "Cost Function active at all times" << std::endl; }
};
}

typedef tpl::TimeActivationBase<double> TimeActivationBase;
}
}
