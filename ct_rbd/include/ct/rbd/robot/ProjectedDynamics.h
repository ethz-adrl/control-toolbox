/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/robot/jacobian/ConstraintJacobian.h>
#include <ct/rbd/robot/kinematics/RBDDataMap.h>
#include <ct/rbd/robot/Kinematics.h>
#include <ct/rbd/state/JointAcceleration.h>
#include <ct/rbd/state/RBDAcceleration.h>
#include <ct/rbd/state/RBDState.h>
#include <ct/rbd/state/RigidBodyAcceleration.h>
#include <ct/rbd/state/RigidBodyPose.h>
#include <ct/rbd/state/RigidBodyState.h>

namespace ct {
namespace rbd {

template <class RBD, size_t NEE>
class ProjectedDynamics
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename RBD::SCALAR Scalar;

    static const size_t NJOINTS = RBD::NJOINTS;
    static const size_t CONTROL_DIM = RBD::NJOINTS;  //this is very big assumption!
    static const size_t NDOF = RBDState<NJOINTS, Scalar>::NDOF;
    static const size_t MAX_JAC_SIZE = 3 * NEE;

    // this typedefs should live somewhere else
    typedef Eigen::Matrix<Scalar, NDOF, 1> g_coordinate_vector_t;
    typedef Eigen::Matrix<Scalar, CONTROL_DIM, 1> control_vector_t;
    typedef Eigen::Matrix<Scalar, NJOINTS, 1> joint_vector_t;
    typedef Eigen::Matrix<Scalar, NDOF, NDOF> inertia_matrix_t;
    typedef Eigen::Matrix<Scalar, 6, 1> ForceVector_t;
    typedef Eigen::Matrix<Scalar, CONTROL_DIM, NDOF> selection_matrix_t;

    typedef RBDDataMap<Eigen::Vector3d, NEE> EE_contact_forces_t;
    typedef RBDDataMap<bool, NEE> EE_in_contact_t;

    typedef RBDState<NJOINTS, Scalar> RBDState_t;
    typedef RBDAcceleration<NJOINTS, Scalar> RBDAcceleration_t;
    typedef JointState<NJOINTS, Scalar> JointState_t;
    typedef JointAcceleration<NJOINTS, Scalar> JointAcceleration_t;

    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorXs;

    /**
	 * @brief The Constructor
	 * @param[in]	kyn     Robot Kinematics
	 * @param[in]  ee_inc  The EE Boolean Data Map with the end effectors in contact
	 */
    ProjectedDynamics(const std::shared_ptr<Kinematics<RBD, NEE>> kyn,
        const EE_in_contact_t ee_inc = EE_in_contact_t(false));

    ~ProjectedDynamics(){};

    /**
	 * @brief set the End Effector contact configuration
	 * @param[in] eeinc The EE Boolean Data Map of the contact configuration
	 */
    void setContactConfiguration(const EE_in_contact_t& eeinc)
    {
        ee_in_contact_ = eeinc;
        ResetJacobianStructure();
        setSizes();
    }

    /**
	 * @brief get the End Effector contact configuration
	 * @param[out]	eeinc	The EE Boolean Data Map of the contact configuration
	 */
    void getContactConfiguration(EE_in_contact_t& eeinc) { eeinc = ee_in_contact_; }
    /**
	 * @brief Computes the forward dynamics in the constraint consistent subspace
	 * of the current contact configuration
	 * param[in] 	x	The RBD state
	 * param[in] 	u	The control vector
	 * param[out]	qdd	The RBD acceleration
	 */
    void ProjectedForwardDynamics(const RBDState_t& x, const control_vector_t& u, RBDAcceleration_t& qdd)
    {
        ProjectedForwardDynamicsCommon(x, u);
        qdd.base().fromVector6d(qddlambda_.template segment<6>(0));
        qdd.joints().setAcceleration(qddlambda_.template segment<NJOINTS>(6));
    }

    /// @brief compute contact forces from last dynamics call
    void getContactForcesInBase(EE_contact_forces_t& lambda)
    {
        int rowCounter = 0;
        for (int eeinc_i = 0; eeinc_i < NEE; eeinc_i++)
        {
            if (ee_in_contact_[eeinc_i])
            {
                lambda[eeinc_i] = qddlambda_.template segment<3>(NDOF + rowCounter);
                rowCounter += 3;
            }
            else
            {
                lambda[eeinc_i].setZero();
            }
        }
    };

    /// @brief compute contact forces at current state and control input
    void getContactForcesInBase(const RBDState_t& x, const control_vector_t& u, EE_contact_forces_t& lambda)
    {
        ProjectedForwardDynamicsCommon(x, u);
        getContactForcesInBase(lambda);
    }

    /**
	 * @brief Computes the Inverse Dynamics in the constraint consistent subspace
	 * of the current contact configuration
	 * param[in]	x	The RBD state
	 * param[in]	qdd	The RBD Acceleration
	 * param[out]	u	The control vector
	 */
    void ProjectedInverseDynamics(const RBDState_t& x, const RBDAcceleration_t& qdd, control_vector_t& u)
    {
        updateDynamicsTerms(x, u);
        ProjectedInverseDynamicsCommon(x, qdd, u);
    }

    void ProjectedInverseDynamicsNoGravity(const RBDState_t& x, const RBDAcceleration_t& qdd, control_vector_t& u)
    {
        updateDynamicsTermsNoGravity(x, u);
        ProjectedInverseDynamicsCommon(x, qdd, u);
    }

private:
    /// @brief Update M h & f terms of the dynamics equation
    void updateDynamicsTerms(const RBDState_t& x, const control_vector_t& u);

    /// @brief Update M h & f terms of the dynamics equation, excluding gravity from h
    void updateDynamicsTermsNoGravity(const RBDState_t& x, const control_vector_t& u);

    /**
     * @brief Solves the equation
     *      P (M*qdd + h) = P*St*tau
     *      with P Jc^T = 0
     *      The user is responsible for providing a constraint consistent acceleration
     */
    void ProjectedInverseDynamicsCommon(const RBDState_t& x, const RBDAcceleration_t& qdd, control_vector_t& u);

    /**
     * @brief Simultaniously solves the equations
     *      M*qdd + h = St*tau + Jct*lambda
     *      Jc*qdd + dJcdt*qd + omega x v = 0  (No acceleration of the feet)
     */
    void ProjectedForwardDynamicsCommon(const RBDState_t& x, const control_vector_t& u);

    void ResetJacobianStructure();

    void setSizes();

    std::shared_ptr<Kinematics<RBD, NEE>> kinematics_; /*!< The RBD kinematics */

    EE_in_contact_t ee_in_contact_; /*!< the contact configuration data map*/

    size_t neec_ = 0; /*!< The number of EE in contact */

    inertia_matrix_t M_;      /*!< The inertia matrix */
    selection_matrix_t S_;    /*!< The selection matrix */
    g_coordinate_vector_t f_; /*!< The input force*/
    g_coordinate_vector_t h_; /*!< the c and g-forces */

    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MJTJ0_;
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> qddlambda_, b_;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Jc_reduced_;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> dJcdt_reduced_;
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> feet_crossproduct_;

    tpl::ConstraintJacobian<Kinematics<RBD, NEE>, MAX_JAC_SIZE, NJOINTS, Scalar>
        Jc_; /*!< The Jacobian of the constraint */

    inertia_matrix_t P_; /*!< The Projector */
    Eigen::JacobiSVD<MatrixXs> svd_;
};

template <class RBD, size_t NEE>
ProjectedDynamics<RBD, NEE>::ProjectedDynamics(const std::shared_ptr<Kinematics<RBD, NEE>> kyn,
    const EE_in_contact_t ee_inc /*= EE_in_Contact_t(false)*/)
    : kinematics_(kyn), ee_in_contact_(ee_inc)
{
    setContactConfiguration(ee_inc);
}

template <class RBD, size_t NEE>
void ProjectedDynamics<RBD, NEE>::updateDynamicsTermsNoGravity(const RBDState_t& x, const control_vector_t& u)
{
    joint_vector_t jForces;
    ForceVector_t base_w;

    kinematics_->robcogen().inverseDynamics().C_terms_fully_actuated(
        base_w, jForces, x.baseVelocities().getVector(), x.joints().getPositions(), x.joints().getVelocities());

    M_ = kinematics_->robcogen().jSim().update(x.joints().getPositions());

    h_ << base_w, jForces;
    f_ << Eigen::Matrix<Scalar, 6, 1>::Zero(), u;
}

template <class RBD, size_t NEE>
void ProjectedDynamics<RBD, NEE>::updateDynamicsTerms(const RBDState_t& x, const control_vector_t& u)
{
    joint_vector_t jForces_gravity;
    ForceVector_t base_w_gravity;

    kinematics_->robcogen().inverseDynamics().G_terms_fully_actuated(
        base_w_gravity, jForces_gravity, x.basePose().computeGravityB6D(), x.joints().getPositions());

    updateDynamicsTermsNoGravity(x, u);
    h_.template segment<6>(0) += base_w_gravity;
    h_.template segment<NJOINTS>(6) += jForces_gravity;
}

template <class RBD, size_t NEE>
void ProjectedDynamics<RBD, NEE>::ProjectedInverseDynamicsCommon(const RBDState_t& x,
    const RBDAcceleration_t& qdd,
    control_vector_t& u)
{
    Jc_.updateState(x);
    P_ = Jc_.P();

    Eigen::Matrix<Scalar, NDOF, CONTROL_DIM> PSt = P_ * S_.transpose();

    svd_.compute(PSt, Eigen::ComputeThinU | Eigen::ComputeThinV);

    VectorXs sing_values(svd_.matrixV().cols(), 1);  // size of E has same size as columns of V
    sing_values = (svd_.singularValues().array() > 1e-9).select(svd_.singularValues().array().inverse(), 0);
    selection_matrix_t Pstinv = svd_.matrixV() * sing_values.asDiagonal() * svd_.matrixU().transpose();

    selection_matrix_t PStP = Pstinv * P_;

    g_coordinate_vector_t Mqdd = M_ * qdd.toCoordinateAcceleration();

    u = PStP * (Mqdd + h_);
}

template <class RBD, size_t NEE>
void ProjectedDynamics<RBD, NEE>::ProjectedForwardDynamicsCommon(const RBDState_t& x, const control_vector_t& u)
{
    // Set Kinematics
    Jc_.updateState(x);
    int rowCount = 0;
    for (size_t eeinc_i = 0; eeinc_i < NEE; eeinc_i++)
    {
        if (ee_in_contact_[eeinc_i])
        {
            Jc_reduced_.template block<3, NDOF>(rowCount, 0) = Jc_.J().template block<3, NDOF>(3 * eeinc_i, 0);
            dJcdt_reduced_.template block<3, NDOF>(rowCount, 0) = Jc_.dJdt().template block<3, NDOF>(3 * eeinc_i, 0);
            feet_crossproduct_.template segment<3>(rowCount) =
                x.baseLocalAngularVelocity().toImplementation().template cross(
                    kinematics_->getEEVelocityInBase(eeinc_i, x).toImplementation());
            rowCount += 3;
        }
    }

    // Set Dynamics
    updateDynamicsTerms(x, u);
    MJTJ0_.template block<NDOF, NDOF>(0, 0) = M_;
    MJTJ0_.template block(NDOF, 0, 3 * neec_, NDOF) = -Jc_reduced_;
    MJTJ0_.template block(0, NDOF, NDOF, 3 * neec_) = -Jc_reduced_.transpose();

    b_.template segment<NDOF>(0) = f_ - h_;
    b_.template segment(NDOF, 3 * neec_) = dJcdt_reduced_ * x.toCoordinateVelocity() + feet_crossproduct_;

    qddlambda_ = core::LDLTsolve<Scalar>(MJTJ0_, b_);
}

template <class RBD, size_t NEE>
void ProjectedDynamics<RBD, NEE>::ResetJacobianStructure()
{
    Jc_.ee_indices_.clear();
    Jc_.c_size_ = 0;
    neec_ = 0;

    for (size_t eeinc = 0; eeinc < NEE; eeinc++)
    {
        if (ee_in_contact_[eeinc])
        {
            neec_++;
            Jc_.c_size_ += 3;
            Jc_.ee_indices_.push_back(eeinc);

            Jc_.eeInContact_[eeinc] = ee_in_contact_[eeinc];
        }
    }
}

template <class RBD, size_t NEE>
void ProjectedDynamics<RBD, NEE>::setSizes()
{
    Jc_reduced_.resize(3 * neec_, NDOF);
    dJcdt_reduced_.resize(3 * neec_, NDOF);
    feet_crossproduct_.resize(3 * neec_);

    MJTJ0_.resize(NDOF + 3 * neec_, NDOF + 3 * neec_);
    MJTJ0_.setZero();
    b_.resize(NDOF + 3 * neec_);
    qddlambda_.resize(NDOF + 3 * neec_);

    S_.template block<CONTROL_DIM, 6>(0, 0).setZero();
    S_.template block<CONTROL_DIM, NJOINTS>(0, 6).setIdentity();
}

} /* namespace rbd */
} /* namespace ct  */
