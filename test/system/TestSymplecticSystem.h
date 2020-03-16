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
class TestSymplecticSystem : public SymplecticSystem<1, 1, 1, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_DIM = 2;
    static const size_t CONTROL_DIM = 1;
    static const size_t POS_DIM = 1;
    static const size_t VEL_DIM = 1;

    TestSymplecticSystem() = delete;

    // constructor directly using frequency and damping coefficients
    TestSymplecticSystem(SCALAR w_n, std::shared_ptr<Controller<2, 1, SCALAR>> controller = nullptr)
        : SymplecticSystem<1, 1, 1, SCALAR>(controller), w_n_(w_n)
    {
    }

    TestSymplecticSystem(const TestSymplecticSystem& arg) : SymplecticSystem<1, 1, 1, SCALAR>(arg), w_n_(arg.w_n_) {}

    virtual ~TestSymplecticSystem() = default;

    TestSymplecticSystem* clone() const override { return new TestSymplecticSystem(*this); }

    //! need to override this method for a symplectic system
    virtual void computePdot(const StateVector<POS_DIM + VEL_DIM, SCALAR>& x,
        const StateVector<VEL_DIM, SCALAR>& v,
        const ControlVector<CONTROL_DIM, SCALAR>& control,
        StateVector<POS_DIM, SCALAR>& pDot) override
    {
        pDot(0) = v(0);
    }

    //! need to override this method for a symplectic system
    virtual void computeVdot(const StateVector<POS_DIM + VEL_DIM, SCALAR>& x,
        const StateVector<POS_DIM, SCALAR>& p,
        const ControlVector<CONTROL_DIM, SCALAR>& control,
        StateVector<VEL_DIM, SCALAR>& vDot) override
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
