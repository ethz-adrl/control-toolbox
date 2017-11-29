/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#pragma once


#include "jacobian/ConstraintJacobian.h"
#include "kinematics/RBDDataMap.h"
#include "Kinematics.h"
#include "state/JointAcceleration.h"
#include "state/RBDAcceleration.h"
#include "state/RBDState.h"
#include "state/RigidBodyAcceleration.h"
#include "state/RigidBodyPose.h"
#include "state/RigidBodyState.h"

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
    static const size_t NDOF = RBDState<NJOINTS>::NDOF;
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

    typedef tpl::RBDState<NJOINTS, Scalar> RBDState_t;
    typedef tpl::RBDAcceleration<NJOINTS, Scalar> RBDAcceleration_t;
    typedef tpl::JointState<NJOINTS, Scalar> JointState_t;
    typedef tpl::JointAcceleration<NJOINTS, Scalar> JointAcceleration_t;

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
        updatePD(x, u);

        g_coordinate_vector_t qd = x.toCoordinateVelocity();

        g_coordinate_vector_t b = fp_ - hp_ - (Cc_ * qd);
        // - M J^-1 J_dot * qd +
        //b = -Cc_ * qd + fp_ - hp_;

        g_coordinate_vector_t y = Mc_.fullPivLu().solve(b);

        //ToDo: better assignment methods
        qdd.base().fromVector6d(y.template segment<6>(0));
        qdd.joints().setAcceleration(y.template segment<NJOINTS>(6));
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
        /// This method is generic for any type of projection, however the current implementation
        /// of the class only provides the orthogonal projector.
        /// ToDo: For the orthogonal projection some of these operations are redundant

        updatePD(x, u);

        Eigen::Matrix<Scalar, NDOF, CONTROL_DIM> PSt = P_ * S_.transpose();

        svd_.compute(PSt, Eigen::ComputeThinU | Eigen::ComputeThinV);

        VectorXs sing_values(svd_.matrixV().cols(), 1);  // size of E has same size as columns of V
        sing_values = (svd_.singularValues().array() > 1e-9).select(svd_.singularValues().array().inverse(), 0);
        selection_matrix_t Pstinv = svd_.matrixV() * sing_values.asDiagonal() * svd_.matrixU().transpose();

        selection_matrix_t PStP = Pstinv * P_;

        g_coordinate_vector_t Mqdd = M_ * qdd.toCoordinateAcceleration();

        u = PStP * (Mqdd + h_);
    }

    void ProjectedInverseDynamicsNoGravity(const RBDState_t& x, const RBDAcceleration_t& qdd, control_vector_t& u)
    {
        /// This method is generic for any type of projection, however the current implementation
        /// of the class only provides the orthogonal projector.
        /// ToDo: For the orthogonal projection some of these operations are redundant

        updatePDNoGravity(x, u);

        Eigen::Matrix<Scalar, NDOF, CONTROL_DIM> PSt = P_ * S_.transpose();

        svd_.compute(PSt, Eigen::ComputeThinU | Eigen::ComputeThinV);

        VectorXs sing_values(svd_.matrixV().cols(), 1);  // size of E has same size as columns of V
        sing_values = (svd_.singularValues().array() > 1e-9).select(svd_.singularValues().array().inverse(), 0);
        selection_matrix_t Pstinv = svd_.matrixV() * sing_values.asDiagonal() * svd_.matrixU().transpose();

        selection_matrix_t PStP = Pstinv * P_;

        g_coordinate_vector_t Mqdd = M_ * qdd.toCoordinateAcceleration();

        u = PStP * (Mqdd + h_);
    }

    /// @brief compute contact forces from last dynamics call given the last
    /// call of the dynamics.
    /// the idea is to allow the user to get the contact forces only if required
    void computeContactForces(EE_contact_forces_t& lamda);

private:
    ///@brief Update constraint dynamics terms
    void updatePD(const RBDState_t& x, const control_vector_t& u);

    void updatePDNoGravity(const RBDState_t& x, const control_vector_t& u);

    ///@brief Update the Projected terms
    void updateProjectedTerms(const control_vector_t& u);

    /// @brief Update M h & f terms of the dynamics equation
    void updateDynamicsTerms(const RBDState_t& x, const control_vector_t& u);

    void updateDynamicsTermsNoGravity(const RBDState_t& x, const control_vector_t& u);

    void ResetJacobianStructure();

private:
    std::shared_ptr<Kinematics<RBD, NEE>> kinematics_; /*!< The RBD kinematics */

    EE_in_contact_t ee_in_contact_; /*!< the contact configuration data map*/

    size_t neec_ = 0; /*!< The number of EE in contact */

    inertia_matrix_t M_;      /*!< The inertia matrix */
    selection_matrix_t S_;    /*!< The selection matrix */
    g_coordinate_vector_t f_; /*!< The input force*/
    g_coordinate_vector_t h_; /*!< the c and g-forces */

    inertia_matrix_t Mc_;      /*!< The constraint inertia matrix */
    g_coordinate_vector_t fp_; /*!< acting input force*/
    g_coordinate_vector_t hp_; /*!< acting forces */
    inertia_matrix_t Cc_;      /*!< Internal forces (ToDo: find better name)*/


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
}

template <class RBD, size_t NEE>
void ProjectedDynamics<RBD, NEE>::updatePD(const RBDState_t& x, const control_vector_t& u)
{
    // currently using the null space orthogonal projection

    updateDynamicsTerms(x, u);
    Jc_.updateState(x);
    updateProjectedTerms(u);
}

template <class RBD, size_t NEE>
void ProjectedDynamics<RBD, NEE>::updatePDNoGravity(const RBDState_t& x, const control_vector_t& u)
{
    // currently using the null space orthogonal projection

    updateDynamicsTermsNoGravity(x, u);
    Jc_.updateState(x);
    updateProjectedTerms(u);
}

template <class RBD, size_t NEE>
void ProjectedDynamics<RBD, NEE>::updateDynamicsTermsNoGravity(const RBDState_t& x, const control_vector_t& u)
{
    joint_vector_t jForces;
    ForceVector_t base_w;

    kinematics_->robcogen().inverseDynamics().C_terms_fully_actuated(
        base_w, jForces, x.baseVelocities().getVector(), x.joints().getPositions(), x.joints().getVelocities());

    M_ = kinematics_->robcogen().jSim().update(x.joints().getPositions());

    // todo replace with selection matrix from kinematics_!
    S_.template block<CONTROL_DIM, 6>(0, 0).setZero();
    S_.template block<CONTROL_DIM, NJOINTS>(0, 6).setIdentity();

    h_ << base_w, jForces;
    f_ << Eigen::Matrix<Scalar, 6, 1>::Zero(), u;
}

template <class RBD, size_t NEE>
void ProjectedDynamics<RBD, NEE>::updateDynamicsTerms(const RBDState_t& x, const control_vector_t& u)
{
    joint_vector_t jForces, jForces_gravity, jForces_velocity;
    ForceVector_t base_w, base_w_gravity, base_w_velocity;


    // Calls set kinematics->setjointstates
    kinematics_->robcogen().inverseDynamics().G_terms_fully_actuated(
        base_w_gravity, jForces_gravity, x.basePose().computeGravityB6D(), x.joints().getPositions());

    kinematics_->robcogen().inverseDynamics().C_terms_fully_actuated(
        base_w_velocity, jForces_velocity, x.baseVelocities().getVector(), x.joints().getVelocities());

    base_w = base_w_gravity + base_w_velocity;
    jForces = jForces_gravity + jForces_velocity;

    M_ = kinematics_->robcogen().jSim().update(x.joints().getPositions());

    // todo replace with selection matrix from kinematics_!
    S_.template block<CONTROL_DIM, 6>(0, 0).setZero();
    S_.template block<CONTROL_DIM, NJOINTS>(0, 6).setIdentity();

    h_ << base_w, jForces;
    f_ << Eigen::Matrix<Scalar, 6, 1>::Zero(), u;
}

template <class RBD, size_t NEE>
void ProjectedDynamics<RBD, NEE>::updateProjectedTerms(const control_vector_t& u)
{
    P_ = Jc_.P();

    /// Constraint Inertia Matrix
    /// There are various versions, user should decide on this?
    ///
    inertia_matrix_t PM = P_ * M_;
    inertia_matrix_t PMT = PM.transpose();
    Mc_ = M_ + PM - PMT;

    ///Acting input force
    fp_ = P_ * S_.transpose() * u;

    ///Acting forces
    hp_ = P_ * h_;

    ///InternalForces
    inertia_matrix_t C = Jc_.JdagerSVD() * Jc_.dJdt();
    Cc_ = M_ * C;
}

template <class RBD, size_t NEE>
void ProjectedDynamics<RBD, NEE>::ResetJacobianStructure()
{
    Jc_.ee_indices_.clear();
    Jc_.c_size_ = 0;
    neec_ = 0;


    for (auto eeinc = 0; eeinc < NEE; eeinc++)
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

} /* namespace rbd */
} /* namespace ct  */
