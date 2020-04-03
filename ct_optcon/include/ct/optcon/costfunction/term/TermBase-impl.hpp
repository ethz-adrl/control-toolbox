/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {


template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::TermBase(std::string name)
    : name_(name),
      c_i_(
          std::shared_ptr<ct::core::tpl::ActivationBase<SCALAR_EVAL>>(new ct::core::tpl::ActivationBase<SCALAR_EVAL>()))
{
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::TermBase(const TermBase& arg) : name_(arg.name_), c_i_(arg.c_i_)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::~TermBase()
{
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
bool TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::isActiveAtTime(SCALAR_EVAL t)
{
    return c_i_->isActive(t);
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
auto TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::computeActivation(const SCALAR_EVAL t) -> SCALAR_EVAL
{
    return c_i_->computeActivation(t);
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
auto TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::stateDerivative(const MANIFOLD& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> core::StateVector<STATE_DIM, SCALAR_EVAL>
{
    throw std::runtime_error(
        "This cost function element is not implemented "
        "for the given term. Please use either auto-diff cost function "
        "or implement the analytical derivatives manually.");
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
auto TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::stateSecondDerivative(const MANIFOLD& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> state_matrix_t
{
    throw std::runtime_error(
        "This cost function element is not implemented "
        "for the given term. Please use either auto-diff cost function or "
        "implement the analytical derivatives manually.");
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
auto TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::controlDerivative(const MANIFOLD& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> core::ControlVector<CONTROL_DIM, SCALAR_EVAL>
{
    throw std::runtime_error(
        "This cost function element is not implemented "
        "for the given term. Please use either auto-diff cost function "
        "or implement the analytical derivatives manually.");
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
auto TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::controlSecondDerivative(const MANIFOLD& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> control_matrix_t
{
    throw std::runtime_error(
        "This cost function element is not implemented "
        "for the given term. Please use either auto-diff cost function "
        "or implement the analytical derivatives manually.");
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
auto TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::stateControlDerivative(const MANIFOLD& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> control_state_matrix_t
{
    throw std::runtime_error(
        "This cost function element is not implemented "
        "for the given term. Please use either auto-diff cost function "
        "or implement the analytical derivatives manually.");
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
void TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::loadConfigFile(const std::string& filename,
    const std::string& termName,
    bool verbose)
{
    throw std::runtime_error(
        "This cost function element is not implemented for the given term. Please use either auto-diff cost function "
        "or implement the analytical derivatives manually.");
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
void TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::setTimeActivation(
    std::shared_ptr<ct::core::tpl::ActivationBase<SCALAR_EVAL>> c_i,
    bool verbose)
{
    c_i_ = c_i;
    if (verbose)
        c_i_->printInfo();
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
void TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::loadTimeActivation(const std::string& filename,
    const std::string& termName,
    bool verbose)
{
    if (verbose)
        std::cout << "TermBase: loading TimeActivation ..." << std::endl;

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    try
    {
        std::string activationKind = pt.get<std::string>(termName + ".time_activation" + ".kind");
        boost::algorithm::to_lower(activationKind);
        std::shared_ptr<ct::core::tpl::ActivationBase<SCALAR_EVAL>> c_i;
        CT_LOADABLE_ACTIVATIONS(SCALAR_EVAL);
        c_i->loadConfigFile(filename, termName + ".time_activation", verbose);
        if (!c_i)
        {
            throw std::runtime_error("Activation type \"" + activationKind + "\" not supported");
        }
        else
        {
            c_i_ = c_i;
            if (verbose)
                c_i_->printInfo();
        }
    } catch (std::exception& e)
    {
        if (verbose)
            std::cout << "TermBase: encountered exception while loading TimeActivation." << std::endl;
        return;
    }
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
const std::string& TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::getName() const
{
    return name_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
void TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::setName(const std::string& termName)
{
    name_ = termName;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
void TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::updateReferenceState(const MANIFOLD& newRefState)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
void TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::updateReferenceControl(
    const ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD>
MANIFOLD TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>::getReferenceState() const
{
    throw std::runtime_error("getReferenceState is not implemented for the current term!");
}


}  // namespace optcon
}  // namespace ct
