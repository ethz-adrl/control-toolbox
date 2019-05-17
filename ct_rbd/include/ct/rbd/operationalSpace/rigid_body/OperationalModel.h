/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "OperationalModelBase.h"

namespace ct {
namespace rbd {

/**
 * \ingroup OS
 * \brief This is the class for the operational space model which gives access to the operational model parameter.
 * The model is assumed to have the following form:
 * \f$ M \ddot{x} + C + G = S^\top \tau + J_c^\top  \lambda \f$
 * where \f$ \ddot{x} \f$ is the acceleration of the operational space coordinate.
 */
template <size_t NUM_OUTPUTS, size_t NUM_JOINTS, size_t NUM_CONTACTPOINTS>
class OperationalModel : public OperationalModelBase<NUM_OUTPUTS, NUM_JOINTS, NUM_CONTACTPOINTS>
{
public:
    typedef std::shared_ptr<OperationalModel<NUM_OUTPUTS, NUM_JOINTS, NUM_CONTACTPOINTS>> ptr;
    typedef typename OperationalModelBase<NUM_JOINTS + 6, NUM_JOINTS, NUM_CONTACTPOINTS>::ptr
        full_body_model_ptr_t;  // this is a pointer type to original system
    typedef OperationalModelBase<NUM_OUTPUTS, NUM_JOINTS, NUM_CONTACTPOINTS> Base;
    typedef typename Base::state_t state_t;
    typedef typename Base::jacobian_t jacobian_t;                          //JacobianType;
    typedef typename Base::jacobian_inv_t jacobian_inv_t;                  //JacobianInverseType;
    typedef typename Base::jacobian_class_ptr_t jacobian_class_ptr_t;      //JacobianClassType;
    typedef typename Base::coordinate_class_ptr_t coordinate_class_ptr_t;  //CoordinateClassType;

    OperationalModel(const full_body_model_ptr_t& fullModelPtr,
        const jacobian_class_ptr_t& jacobianPtr,
        const coordinate_class_ptr_t& coordinatePtr = nullptr)
        : Base(jacobianPtr, coordinatePtr), fullModelPtr_(fullModelPtr), jacobianPtr_(jacobianPtr)
    {
        if (!fullModelPtr)
            throw std::runtime_error("Model pointer in is not instantiated!");
        if (!jacobianPtr)
            throw std::runtime_error("Task space Jacobian pointer in is not instantiated!");
    }

    OperationalModel() = delete;
    ~OperationalModel(){};

    /**
	 * This method updates the class member variables using the current state.
	 * @param state The current state
	 */
    void update(const state_t& state) override
    {
        Base::state_ = state;

        fullModelPtr_->update(state);
        Base::jacobianPtr_->updateState(state, fullModelPtr_->MInverse());

        // jacobian dager
        const jacobian_inv_t& Jdager = Base::jacobianPtr_->JdagerLDLT();

        // rigid body coefficients
        Base::M_ = Jdager.transpose() * fullModelPtr_->M() * Jdager;
        Base::MInverse_ = Base::M_.ldlt().solve(Eigen::MatrixXd::Identity(NUM_OUTPUTS, NUM_OUTPUTS));
        Base::C_ = Jdager.transpose() * fullModelPtr_->C() -
                   Base::M_ * Base::jacobianPtr_->dJdt() * state.toCoordinateVelocity();
        Base::G_ = Jdager.transpose() * fullModelPtr_->G();
        Base::S_ = fullModelPtr_->S() * Jdager;
        // contact jacobians
        for (size_t i = 0; i < NUM_CONTACTPOINTS; i++)
            Base::AllJc_[i] = fullModelPtr_->Jc(i) * Jdager;
    }

    /**
	 * this method returns a pointer to the internal operational space Jacobian.
	 * @return pointer to the internal operational space Jacobian
	 */
    jacobian_class_ptr_t getJacobianPtr() { return jacobianPtr_; }
    const typename Base::S_t& STransposeDager(
        const Eigen::MatrixXd& orthogonalSystemS = Eigen::MatrixXd::Zero(NUM_JOINTS, 0))
    {
        Eigen::Matrix<double, NUM_JOINTS, NUM_JOINTS> fixedBaseMassMatrix =
            (fullModelPtr_->S() * fullModelPtr_->MInverse() * fullModelPtr_->S().transpose())
                .ldlt()
                .solve(Eigen::MatrixXd::Identity(NUM_JOINTS, NUM_JOINTS));

        Eigen::Matrix<double, NUM_JOINTS, NUM_JOINTS> W0 = fixedBaseMassMatrix;
        W0.setIdentity();

        Eigen::Matrix<double, NUM_JOINTS, NUM_JOINTS> W;
        if (orthogonalSystemS.cols() != 0)
            W = W0 -
                W0 * orthogonalSystemS *
                    (orthogonalSystemS.transpose() * W0 * orthogonalSystemS)
                        .ldlt()
                        .solve(Eigen::MatrixXd::Identity(orthogonalSystemS.cols(), orthogonalSystemS.cols())) *
                    orthogonalSystemS.transpose() * W0;
        else
            W = W0;

        STransposeDager_ =
            W * Base::S_ *
            (Base::S_.transpose() * W * Base::S_).ldlt().solve(Eigen::MatrixXd::Identity(NUM_OUTPUTS, NUM_OUTPUTS));
        return STransposeDager_;
    }

private:
    full_body_model_ptr_t fullModelPtr_;
    jacobian_class_ptr_t jacobianPtr_;
    typename Base::S_t STransposeDager_;
};


template <>
inline void OperationalModel<0, 12, 4>::update(const state_t& state)
{
    Base::state_ = state;
    fullModelPtr_->update(state);
}

template <>
inline const OperationalModel<0, 12, 4>::Base::S_t& OperationalModel<0, 12, 4>::STransposeDager(
    const Eigen::MatrixXd& orthogonalSystemS)
{
    STransposeDager_.setZero();
    return STransposeDager_;
}
}
}
