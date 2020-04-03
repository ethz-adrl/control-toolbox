/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <typename MANIFOLD, size_t CONTROL_DIM>
TermBase<MANIFOLD, CONTROL_DIM>::TermBase(std::string name)
    : name_(name),
      c_i_(
          std::shared_ptr<ct::core::tpl::ActivationBase<SCALAR_EVAL>>(new ct::core::tpl::ActivationBase<SCALAR_EVAL>()))
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
TermBase<MANIFOLD, CONTROL_DIM>::TermBase(const TermBase& arg) : name_(arg.name_), c_i_(arg.c_i_)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
TermBase<MANIFOLD, CONTROL_DIM>::~TermBase()
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
bool TermBase<MANIFOLD, CONTROL_DIM>::isActiveAtTime(SCALAR_EVAL t)
{
    return c_i_->isActive(t);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto TermBase<MANIFOLD, CONTROL_DIM>::computeActivation(const SCALAR_EVAL t) -> SCALAR_EVAL
{
    return c_i_->computeActivation(t);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto TermBase<MANIFOLD, CONTROL_DIM>::stateDerivative(const EVAL_MANIFOLD& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> core::StateVector<STATE_DIM, SCALAR_EVAL>
{
    throw std::runtime_error(
        "This cost function element is not implemented "
        "for the given term. Please use either auto-diff cost function "
        "or implement the analytical derivatives manually.");
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto TermBase<MANIFOLD, CONTROL_DIM>::stateSecondDerivative(const EVAL_MANIFOLD& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> state_matrix_t
{
    throw std::runtime_error(
        "This cost function element is not implemented "
        "for the given term. Please use either auto-diff cost function or "
        "implement the analytical derivatives manually.");
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto TermBase<MANIFOLD, CONTROL_DIM>::controlDerivative(const EVAL_MANIFOLD& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> core::ControlVector<CONTROL_DIM, SCALAR_EVAL>
{
    throw std::runtime_error(
        "This cost function element is not implemented "
        "for the given term. Please use either auto-diff cost function "
        "or implement the analytical derivatives manually.");
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto TermBase<MANIFOLD, CONTROL_DIM>::controlSecondDerivative(const EVAL_MANIFOLD& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> control_matrix_t
{
    throw std::runtime_error(
        "This cost function element is not implemented "
        "for the given term. Please use either auto-diff cost function "
        "or implement the analytical derivatives manually.");
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto TermBase<MANIFOLD, CONTROL_DIM>::stateControlDerivative(const EVAL_MANIFOLD& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t) -> control_state_matrix_t
{
    throw std::runtime_error(
        "This cost function element is not implemented "
        "for the given term. Please use either auto-diff cost function "
        "or implement the analytical derivatives manually.");
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void TermBase<MANIFOLD, CONTROL_DIM>::loadConfigFile(const std::string& filename,
    const std::string& termName,
    bool verbose)
{
    throw std::runtime_error(
        "This cost function element is not implemented for the given term. Please use either auto-diff cost function "
        "or implement the analytical derivatives manually.");
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void TermBase<MANIFOLD, CONTROL_DIM>::setTimeActivation(std::shared_ptr<ct::core::tpl::ActivationBase<SCALAR_EVAL>> c_i,
    bool verbose)
{
    c_i_ = c_i;
    if (verbose)
        c_i_->printInfo();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void TermBase<MANIFOLD, CONTROL_DIM>::loadTimeActivation(const std::string& filename,
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

template <typename MANIFOLD, size_t CONTROL_DIM>
const std::string& TermBase<MANIFOLD, CONTROL_DIM>::getName() const
{
    return name_;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void TermBase<MANIFOLD, CONTROL_DIM>::setName(const std::string& termName)
{
    name_ = termName;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void TermBase<MANIFOLD, CONTROL_DIM>::updateReferenceState(const EVAL_MANIFOLD& newRefState)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void TermBase<MANIFOLD, CONTROL_DIM>::updateReferenceControl(const ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto TermBase<MANIFOLD, CONTROL_DIM>::getReferenceState() const -> EVAL_MANIFOLD
{
    throw std::runtime_error("getReferenceState is not implemented for the current term!");
}

}  // namespace optcon
}  // namespace ct
