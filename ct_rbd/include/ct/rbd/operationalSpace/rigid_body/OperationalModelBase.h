/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once


#include <ct/rbd/robot/jacobian/OperationalJacobianBase.h>
#include <ct/rbd/state/RBDState.h>
#include "../coordinate/CoordinateBase.h"

namespace ct {
namespace rbd {

/**
 * \defgroup OS OperationalSpace
 * \brief Operational Space module is a group of classes for implementing the operational space model.
 */

/**
 * \ingroup OS
 * \Brief  This is the base class for the operational space model which gives access to the model parameter.
 * The model is assumed to have the following form:
 * \f$ M \ddot{x} + C + G = S^\top \tau + J_c^\top  \lambda \f$
 * where \f$ \ddot{x} \f$ is the acceleration of the operational space coordinate.
 */
template <size_t NUM_OUTPUTS, size_t NUM_JOINTS, size_t NUM_CONTACTPOINTS>
class OperationalModelBase
{
public:
    typedef std::shared_ptr<OperationalModelBase<NUM_OUTPUTS, NUM_JOINTS, NUM_CONTACTPOINTS>> ptr;

    typedef typename CoordinateBase<NUM_OUTPUTS, NUM_JOINTS>::ptr coordinate_class_ptr_t;         //CoordinateClassType;
    typedef typename OperationalJacobianBase<NUM_OUTPUTS, NUM_JOINTS>::ptr jacobian_class_ptr_t;  //JacobianClassType;

    typedef typename OperationalJacobianBase<NUM_OUTPUTS, NUM_JOINTS>::jacobian_t jacobian_t;  //JacobianType;
    typedef typename OperationalJacobianBase<NUM_OUTPUTS, NUM_JOINTS>::jacobian_inv_t
        jacobian_inv_t;  //JacobianInverseType;

    typedef RBDState<NUM_JOINTS> state_t;
    typedef Eigen::Matrix<double, NUM_OUTPUTS, NUM_OUTPUTS> M_t;
    typedef Eigen::Matrix<double, NUM_OUTPUTS, 1> C_t;
    typedef Eigen::Matrix<double, NUM_OUTPUTS, 1> G_t;
    typedef Eigen::Matrix<double, NUM_JOINTS, NUM_OUTPUTS> S_t;
    typedef Eigen::Matrix<double, 3, NUM_OUTPUTS> Jc_t;

    OperationalModelBase(const jacobian_class_ptr_t& jacobianPtr = nullptr,
        const coordinate_class_ptr_t& coordinatePtr = nullptr)
        : jacobianPtr_(jacobianPtr), coordinatePtr_(coordinatePtr){};

    virtual ~OperationalModelBase(){};

    //!  Get the inertia matrix: M
    const M_t& M() const { return M_; }
    //!  Get the inertia matrix inverse: M^{-1}
    const M_t& MInverse() const { return MInverse_; }
    //!  Get the Coriolis force vector: C
    const C_t& C() const { return C_; }
    //!  Get the gravity force vector: G
    const G_t& G() const { return G_; }
    //!  Get the selection matrix: S
    const S_t& S() const { return S_; }
    //!  Get all of the contact Jacobians: Jc
    const std::array<Jc_t, NUM_CONTACTPOINTS>& AllJc() const { return AllJc_; }
    //!  Get the ith contact Jacobian: Jc[i]
    const Jc_t& Jc(size_t i) const { return AllJc_[i]; }
    //!  Get the coordinate position
    Eigen::Matrix<double, NUM_OUTPUTS, 1> getPositions()
    {
        if (coordinatePtr_)
            return coordinatePtr_->getCoordinate(state_);
        else
            return state_.toCoordinatePosition().template head<NUM_OUTPUTS>();
    }

    //!  Get the coordinate velocity
    Eigen::Matrix<double, NUM_OUTPUTS, 1> getVelocities()
    {
        if (jacobianPtr_)
            return jacobianPtr_->J() * state_.toCoordinateVelocity();
        else
            return state_.toCoordinateVelocity().template head<NUM_OUTPUTS>();
    }

    //!  Get the coordinate acceleration
    Eigen::Matrix<double, NUM_OUTPUTS, 1> getAccelerations(Eigen::Matrix<double, NUM_JOINTS + 6, 1> qdd)
    {
        if (jacobianPtr_)
            return jacobianPtr_->J() * qdd + jacobianPtr_->dJdt() * state_.toCoordinateVelocity();
        else
            return qdd.template head<NUM_OUTPUTS>();
    }

    /**
	 * This method updates the class member variables using the current state.
	 * @param state The current state
	 */
    virtual void update(const state_t& state) = 0;

protected:
    jacobian_class_ptr_t jacobianPtr_;
    coordinate_class_ptr_t coordinatePtr_;

    state_t state_;

    M_t M_;
    M_t MInverse_;
    C_t C_;
    G_t G_;
    S_t S_;
    std::array<Jc_t, NUM_CONTACTPOINTS> AllJc_;
};

}  // end of rbd namespace
}  // end of os namespace
