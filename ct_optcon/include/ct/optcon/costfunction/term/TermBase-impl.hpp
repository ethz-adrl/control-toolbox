/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
 * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************************************************************************************/

#pragma once

namespace ct {
namespace optcon {


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermBase(std::string name)
    : name_(name),
      c_i_(std::shared_ptr<tpl::TimeActivationBase<SCALAR_EVAL>>(new tpl::TimeActivationBase<SCALAR_EVAL>()))
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermBase(const TermBase& arg) : name_(arg.name_), c_i_(arg.c_i_)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::~TermBase()
{
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
SCALAR_EVAL TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::eval(
    const Eigen::Matrix<SCALAR_EVAL, STATE_DIM, 1>& x,
    const Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, 1>& u,
    const SCALAR_EVAL& t)
{
    return computeActivation(t) * evaluate(x, u, t);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
ct::core::ADCGScalar TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::evaluateCppadCg(
    const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
    const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
    ct::core::ADCGScalar t)
{
    throw std::runtime_error("The cost function term term " + name_ + " does not implement evaluate CppadCg.");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
bool TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::isActiveAtTime(SCALAR_EVAL t)
{
    return c_i_->isActiveAtTime(t);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
SCALAR_EVAL TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::computeActivation(SCALAR_EVAL t)
{
    return c_i_->computeActivation(t);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
core::StateVector<STATE_DIM, SCALAR_EVAL> TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    throw std::runtime_error(
        "This cost function element is not implemented "
        "for the given term. Please use either auto-diff cost function "
        "or implement the analytical derivatives manually.");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::state_matrix_t
TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateSecondDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    throw std::runtime_error(
        "This cost function element is not implemented "
        "for the given term. Please use either auto-diff cost function or "
        "implement the analytical derivatives manually.");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
core::ControlVector<CONTROL_DIM, SCALAR_EVAL> TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::controlDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    throw std::runtime_error(
        "This cost function element is not implemented "
        "for the given term. Please use either auto-diff cost function "
        "or implement the analytical derivatives manually.");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::control_matrix_t
TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::controlSecondDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    throw std::runtime_error(
        "This cost function element is not implemented "
        "for the given term. Please use either auto-diff cost function "
        "or implement the analytical derivatives manually.");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::control_state_matrix_t
TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateControlDerivative(
    const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    const SCALAR_EVAL& t)
{
    throw std::runtime_error(
        "This cost function element is not implemented "
        "for the given term. Please use either auto-diff cost function "
        "or implement the analytical derivatives manually.");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::loadConfigFile(const std::string& filename,
    const std::string& termName,
    bool verbose)
{
    throw std::runtime_error(
        "This cost function element is not implemented for the given term. Please use either auto-diff cost function "
        "or implement the analytical derivatives manually.");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::setTimeActivation(
    std::shared_ptr<tpl::TimeActivationBase<SCALAR_EVAL>> c_i,
    bool verbose)
{
    c_i_ = c_i;
    if (verbose)
        c_i_->printInfo();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::loadTimeActivation(const std::string& filename,
    const std::string& termName,
    bool verbose)
{
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    try
    {
        std::string activationKind = pt.get<std::string>(termName + ".time_activation" + ".kind");
        boost::algorithm::to_lower(activationKind);
        std::shared_ptr<tpl::TimeActivationBase<SCALAR_EVAL>> c_i;
        CT_LOADABLE_TIME_ACTIVATIONS(SCALAR_EVAL);
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
        return;
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
const std::string& TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::getName() const
{
    return name_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::setName(const std::string& termName)
{
    name_ = termName;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::updateReferenceState(
    const Eigen::Matrix<SCALAR_EVAL, STATE_DIM, 1>& newRefState)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
Eigen::Matrix<SCALAR_EVAL, STATE_DIM, 1> TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::getReferenceState()
    const
{
    throw std::runtime_error("getReferenceState is not implemented for the current term!");
}


}  // namespace optcon
}  // namespace ct
