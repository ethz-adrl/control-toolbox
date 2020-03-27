/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {
namespace tpl {

template <typename SCALAR>
SecondOrderSystem<SCALAR>::SecondOrderSystem(SCALAR w_n,
    SCALAR zeta,
    SCALAR g_dc,
    std::shared_ptr<Controller_t> controller)
    : Base(controller, SYSTEM_TYPE::SECOND_ORDER), w_n_(w_n), w_n_square_(w_n_ * w_n_), zeta_(zeta), g_dc_(g_dc)
{
}

template <typename SCALAR>
SecondOrderSystem<SCALAR>::SecondOrderSystem(const SecondOrderSystem& arg)
    : Base(arg), w_n_(arg.w_n_), w_n_square_(arg.w_n_square_), zeta_(arg.zeta_), g_dc_(arg.g_dc_)
{
}

template <typename SCALAR>
SecondOrderSystem<SCALAR>::SecondOrderSystem(SCALAR k,
    SCALAR m,
    SCALAR d,
    SCALAR g_dc,
    std::shared_ptr<Controller_t> controller)
    : Base(controller), w_n_(std::sqrt(k / m)), w_n_square_(w_n_ * w_n_), zeta_(d / (2.0 * m * k)), g_dc_(g_dc)
{
}

template <typename SCALAR>
SecondOrderSystem<SCALAR>* SecondOrderSystem<SCALAR>::clone() const
{
    return new SecondOrderSystem(*this);
}

template <typename SCALAR>
SecondOrderSystem<SCALAR>::~SecondOrderSystem()
{
}

template <typename SCALAR>
void SecondOrderSystem<SCALAR>::setDynamics(SCALAR w_n, SCALAR zeta, SCALAR g_dc)
{
    w_n_ = w_n;
    w_n_square_ = w_n_ * w_n_;
    zeta_ = zeta;
    g_dc_ = g_dc;
}

template <typename SCALAR>
void SecondOrderSystem<SCALAR>::computeControlledDynamics(const StateVector_t& state,
    const Time_t& t,
    const ControlVector<1, SCALAR>& control,
    typename StateVector_t::Tangent& derivative)
{
    derivative(0) = state(1);
    derivative(1) = g_dc_ * control(0) - 2.0 * zeta_ * w_n_ * state(1) - w_n_square_ * state(0);
}

template <typename SCALAR>
bool SecondOrderSystem<SCALAR>::checkParameters()
{
    if (zeta_ < 0)
    {
        std::cout << "Warning: Damping is negative!" << std::endl;
        return false;
    }
    if (w_n_ < 0)
    {
        std::cout << "Warning: Frequency w_n is negative!" << std::endl;
        return false;
    }
    if (g_dc_ < 0)
    {
        std::cout << "Warning: Steady state gain is negative!" << std::endl;
        return false;
    }
    if (g_dc_ == 0)
    {
        std::cout << "Warning: Steady state gain is zero!" << std::endl;
        return false;
    }

    return true;
}

template <typename SCALAR>
void SecondOrderSystem<SCALAR>::printSystemInfo()
{
    std::cout << "Frequency: " << w_n_ << std::endl;
    std::cout << "Zeta: " << zeta_ << std::endl;
    std::cout << "DC gain: " << g_dc_ << std::endl;

    std::cout << "System is ";
    if (zeta_ == 0.0)
    {
        std::cout << "undamped" << std::endl;
    }
    if (zeta_ == 1.0)
    {
        std::cout << "critically damped" << std::endl;
    }
    if (zeta_ > 1.0)
    {
        std::cout << "overdamped" << std::endl;
    }
    if (zeta_ < 1.0)
    {
        std::cout << "underdamped" << std::endl;
    }
}

}  // namespace tpl
}  // namespace core
}  // namespace ct
