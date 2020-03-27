/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "ControlledSystem.h"

namespace ct {
namespace core {

/**
 * @brief      The base class for the implementation of a symplectic system. In
 *             a symplectic system, the position and the velocity update can be
 *             separated. During integration, the velocity gets update first and
 *             the position update uses the updated velocity
 *
 * @tparam     POS_DIM      The position dimension
 * @tparam     VEL_DIM      The velocity dimension
 * @tparam     CONTROL_DIM  The control dimension
 * @tparam     SCALAR       The scalar type
 */
template <typename SYM_MFD, size_t CONTROL_DIM>
class SymplecticSystem : public ControlledSystem<SYM_MFD, CONTROL_DIM, CONTINUOUS_TIME>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr size_t STATE_DIM = SYM_MFD::TangentDim;
    static constexpr size_t POS_DIM = SYM_MFD::PosDim;
    static constexpr size_t VEL_DIM = SYM_MFD::VelDim;

    static_assert(is_symplectic<SYM_MFD>::value, "Symplectic system: manifold must be defined as symplectic.");

    static_assert(STATE_DIM == (POS_DIM + VEL_DIM),
        "Symplectic system: state_dim must be the sum of position and velocity dim.");

    using Base = ControlledSystem<SYM_MFD, CONTROL_DIM, CONTINUOUS_TIME>;
    using Time_t = typename Base::Time_t;
    using Controller_t = Controller<SYM_MFD, CONTROL_DIM, CONTINUOUS_TIME>;
    using SCALAR = typename SYM_MFD::Scalar;

    /**
	 * @brief      Constructor
	 *
	 * @param[in]  type  The type of the system
	 */
    SymplecticSystem(const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL) : Base(type) {}
    /**
	 * @brief      Constructor
	 *
	 * @param[in]  controller  The controller used when integrating the system
	 * @param[in]  type        The type of the system
	 */
    SymplecticSystem(std::shared_ptr<Controller_t> controller, const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL)
        : Base(controller, type)
    {
    }

    /**
	 * @brief      Copy constructor
	 *
	 * @param[in]  arg   The argument
	 */
    SymplecticSystem(const SymplecticSystem& arg) : Base(arg) {}
    /**
	 * @brief      Destructor
	 */
    virtual ~SymplecticSystem() {}
    /**
	 * @brief      Creates a new instance of the object with same properties than original.
	 *
	 * @return     Copy of this object.
	 */
    virtual SymplecticSystem<SYM_MFD, CONTROL_DIM>* clone() const override = 0;

    virtual void computeControlledDynamics(const SYM_MFD& state,
        const Time_t& t,
        const ControlVector<CONTROL_DIM, SCALAR>& control,
        typename SYM_MFD::Tangent& derivative) override
    {
        StateVector<POS_DIM, SCALAR> pDot;
        StateVector<VEL_DIM, SCALAR> vDot;

        computePdot(state, state.tail(VEL_DIM), control, pDot);
        computeVdot(state, state.head(POS_DIM), control, vDot);

        derivative << pDot, vDot;
    }

    /**
	 * @brief      Computes the derivative of the position
	 *
	 * @param[in]  x     The full state vector
	 * @param[in]  v     The updated velocity
	 * @param[out] pDot  The derivative of the position
	 */
    void computePdot(const SYM_MFD& x, const typename SYM_MFD::VelTangent& v, typename SYM_MFD::PosTangent& pDot)
    {
        ControlVector<CONTROL_DIM, SCALAR> controlAction;
        if (this->controller_)
            this->controller_->computeControl(x, 0.0, controlAction);
        else
            controlAction.setZero();

        computePdot(x, v, controlAction, pDot);
    }

    /**
	 * @brief      Computes the derivative of the velocity
	 *
	 * @param[in]  x     The full state vector
	 * @param[in]  p     The position
	 * @param[out]      vDot  The derivative of the velocity
	 */
    void computeVdot(const SYM_MFD& x, const typename SYM_MFD::PosTangent& p, typename SYM_MFD::VelTangent& vDot)
    {
        ControlVector<CONTROL_DIM, SCALAR> controlAction;
        if (this->controller_)
            this->controller_->computeControl(x, 0.0, controlAction);
        else
            controlAction.setZero();

        computeVdot(x, p, controlAction, vDot);
    }

    /**
	 * @brief      Computes the derivative of the position
	 *
	 * @param[in]  x        The full state vector
	 * @param[in]  v        The updated velocity
	 * @param[in]  control  The control input
	 * @param[out] pDot     The derivative of the position
	 */
    virtual void computePdot(const SYM_MFD& x,
        const typename SYM_MFD::VelTangent& v,
        const ControlVector<CONTROL_DIM, SCALAR>& control,
        typename SYM_MFD::PosTangent& pDot) = 0;

    /**
	 * @brief      Computes the derivative of the velocity
	 *
	 * @param[in]  x        The full state vector
	 * @param[in]  p        The position
	 * @param[in]  control  The control input
	 * @param[out] vDot     The derivative of the velocity
	 */
    virtual void computeVdot(const SYM_MFD& x,
        const typename SYM_MFD::PosTangent& p,
        const ControlVector<CONTROL_DIM, SCALAR>& control,
        typename SYM_MFD::VelTangent& vDot) = 0;
};

}  // namespace core
}  // namespace ct
