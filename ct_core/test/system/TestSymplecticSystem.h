/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <cmath>
#include <memory>
#include <iostream>

namespace ct {
namespace core {
namespace tpl {

/*!
 * This is a simple second-order oscillator without damping, which can easily be formulated as symplectic system.
 */
template <typename SCALAR>
class TestSymplecticSystem : public SymplecticSystem<EuclideanStateSymplectic<1, 1, SCALAR>, 1>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using State = EuclideanStateSymplectic<1, 1, SCALAR>;
    using Base = SymplecticSystem<State, 1>;

    static constexpr size_t STATE_DIM = 2;
    static constexpr size_t CONTROL_DIM = 1;
    static constexpr size_t POS_DIM = 1;
    static constexpr size_t VEL_DIM = 1;

    TestSymplecticSystem() = delete;

    // constructor directly using frequency and damping coefficients
    TestSymplecticSystem(SCALAR w_n,
        std::shared_ptr<Controller<State, CONTROL_DIM, CONTINUOUS_TIME>> controller = nullptr)
        : Base(controller), w_n_(w_n)
    {
    }

    TestSymplecticSystem(const TestSymplecticSystem& arg) : Base(arg), w_n_(arg.w_n_) {}
    virtual ~TestSymplecticSystem() = default;

    TestSymplecticSystem* clone() const override { return new TestSymplecticSystem(*this); }

    virtual void computePdot(const State& x,
        const typename State::VelTangent& v,
        const ControlVector<CONTROL_DIM, SCALAR>& control,
        typename State::PosTangent& pDot) override
    {
        pDot(0) = v(0);
    }

    virtual void computeVdot(const State& x,
        const typename State::PosTangent& p,
        const ControlVector<CONTROL_DIM, SCALAR>& control,
        typename State::VelTangent& vDot) override
    {
        vDot(0) = control(0) - w_n_ * w_n_ * p(0);
    }

private:
    SCALAR w_n_;
};

}  // namespace tpl

typedef tpl::TestSymplecticSystem<double> TestSymplecticSystem;

}  // namespace core
}  // namespace ct
