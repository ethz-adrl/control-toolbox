/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/state/RBDState.h>

#pragma GCC diagnostic push  // include IIT headers and disable warnings
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-value"
#include <iit/rbd/rbd.h>
#include <iit/rbd/robcogen_commons.h>
#include <iit/rbd/traits/TraitSelector.h>
#pragma GCC diagnostic pop

namespace ct {
namespace rbd {


/*!
 * \brief A soft contact model that only uses end-effector positions/velocities to compute the contact force
 *
 * This contact model computes forces/torques at the endeffectors given the current state of the robot expressed
 * in generalized coordinates
 *
 * \f[ {}_W \lambda = f(q, \dot{q}) \f]
 *
 * The contact model assumes a plane with fixed orientation located at the origin (0, 0, 0). The contact dynamics
 * are a combination of a spring-damper perpendicular and a damper in parallel to the surface. The force expressed
 * in world coordinates without velocity smoothing or normal force smoothing is defined as
 *
 * \f[ {}_W \lambda = f(q, \dot{q}) = - k ({}_W x_z - z_{offset}) - d {}_W \dot{x}  \f]
 *
 * where \f$ {}_W x_z \f$ is the ground penetration with respect to an offset \f$ z_{offset} \f$ expressed in world
 * coordinates and \f$ \dot{x} \f$ is the velocity of the endeffector.
 *
 * In case normal force smoothing is activated, the first term becomes
 *
 * \f[ {}_W \lambda_n =  k e^{-\alpha_n {}_W x_z} \f]
 *
 * In case velocity smoothing is activated, the second term becomes
 *
 * \f[ {}_W \lambda_t =  s({}_W x_z, \dot{x}) ~  d {}_W \dot{x}_{xy} \f]
 *
 * where the smoothing coefficient is one of the following
 *
 * 1. sigmoid
 * \f[  s({}_W x_z, \dot{x}) = \frac{1}{1 + e^{{}_W x_z \alpha}} \f]
 *
 * 2. tanh (same as sigmoid but computed differently)
 * \f[  s({}_W x_z, \dot{x}) = \frac{1}{2} tanh(-\frac{1}{2} {}_W x_z \alpha) + \frac{1}{2} \f]
 *
 * 3. abs
 * \f[  s({}_W x_z, \dot{x}) = - \frac{1}{2} \frac{{}_W x_z \alpha}{1 + abs(-{}_W x_z \alpha)} + \frac{1}{2} \f]
 *
 * \tparam Kinematics the Kinematics implementation of the robot
 */
template <class Kinematics>
class EEContactModel
{
public:
    static const size_t NUM_EE = Kinematics::NUM_EE;
    static const size_t NJOINTS = Kinematics::NJOINTS;

    typedef typename Kinematics::SCALAR SCALAR;

    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait TRAIT;

    typedef std::array<bool, NUM_EE> ActiveMap;
    typedef typename Kinematics::EEForceLinear EEForceLinear;
    typedef std::array<EEForceLinear, NUM_EE> EEForcesLinear;

    typedef Eigen::Matrix<SCALAR, 3, 1> Vector3s;
    typedef kindr::Position<SCALAR, 3> Position3S;
    typedef kindr::Velocity<SCALAR, 3> Velocity3S;


    /*!
	 * \brief the type of velcity smoothing
	 */
    enum VELOCITY_SMOOTHING
    {
        NONE = 0,     //!< none
        SIGMOID = 1,  //!< sigmoid
        TANH = 2,     //!< tanh
        ABS = 3       //!< abs
    };

    /*!
	 * \brief Default constructor
	 * @param k stiffness of vertical spring
	 * @param d damper coefficient
	 * @param alpha velocity smoothing coefficient
	 * @param alpha_n normal force smoothing coefficient
	 * @param zOffset z offset of the plane with respect to (0, 0, 0)
	 * @param smoothing the type of velocity smoothing
	 * @param kinematics instance of the kinematics (optionally). Should be provided for efficiency when using Auto-Diff.
	 */
    EEContactModel(const SCALAR& k = SCALAR(5000),
        const SCALAR& d = SCALAR(500.0),
        const SCALAR& alpha = SCALAR(100.0),
        const SCALAR& alpha_n = SCALAR(-1.0),
        const SCALAR& zOffset = SCALAR(0.0),
        const VELOCITY_SMOOTHING& smoothing = NONE,
        const std::shared_ptr<Kinematics>& kinematics = std::shared_ptr<Kinematics>(new Kinematics()))
        : kinematics_(kinematics),
          smoothing_(smoothing),
          k_(k),
          d_(d),
          alpha_(alpha),
          alpha_n_(alpha_n),
          zOffset_(zOffset)
    {
        for (size_t i = 0; i < NUM_EE; i++)
            EEactive_[i] = true;
    }

    /*!
	 * \brief Clone operator
	 * @param other instance to clone
	 */
    EEContactModel(const EEContactModel& other)
        : kinematics_(other.kinematics_->clone()),
          smoothing_(other.smoothing_),
          k_(other.k_),
          d_(other.d_),
          alpha_(other.alpha_),
          alpha_n_(other.alpha_n_),
          zOffset_(other.zOffset_),
          EEactive_(other.EEactive_)
    {
    }

    EEContactModel* clone() const { return new EEContactModel(*this); }
    /**
	 * \brief Sets which end-effectors can have forces excerted on them
	 * @param activeMap flags of active end-effectors
	 */
    void setActiveEE(const ActiveMap& activeMap) { EEactive_ = activeMap; }
    /**
	 * \brief Computes the contact forces given a state of the robot. Returns forces expressed in the world frame
	 * @param state The state of the robot
	 * @return End-effector forces expressed in the world frame
	 */
    EEForcesLinear computeContactForces(const RBDState<NJOINTS, SCALAR>& state)
    {
        EEForcesLinear eeForces;

        for (size_t i = 0; i < NUM_EE; i++)
        {
            if (EEactive_[i])
            {
                Vector3s eePenetration = computePenetration(i, state.basePose(), state.jointPositions());

                if (eeInContact(eePenetration))
                {
                    Velocity3S eeVelocity = kinematics_->getEEVelocityInWorld(i, state);
                    eeForces[i] = computeEEForce(eePenetration, eeVelocity);
                }
                else
                {
                    eeForces[i].setZero();
                }
            }
        }

        return eeForces;
    }

    SCALAR& alpha() { return alpha_; }
    SCALAR& alpha_n() { return alpha_n_; }
    SCALAR& k() { return k_; }
    SCALAR& d() { return d_; }
    SCALAR& zOffset() { return zOffset_; }
    VELOCITY_SMOOTHING& smoothing() { return smoothing_; }

private:
    /**
	 * \brief Checks if end-effector is in contact. Currently assumes this is the case for negative z
	 * @param eePenetration The surface penetration of the end-effector
	 * @return flag if the end-effector is in contact
	 */
    bool eeInContact(const Vector3s& eePenetration)
    {
        if (smoothing_ == NONE && eePenetration(2) > 0.0)
            return false;
        else
            return true;
    }


    /**
	 * \brief Computes the surface penetration. Currently assumes the surface is at height z = 0.
	 * @param eeId ID of the end-effector
	 * @param basePose Position of the robot base
	 * @param jointPosition Joint position of the robot
	 * @return Penetration in world coordinates
	 */
    Vector3s computePenetration(const size_t& eeId,
        const tpl::RigidBodyPose<SCALAR>& basePose,
        const typename JointState<NJOINTS, SCALAR>::Position& jointPosition)
    {
        Position3S pos = kinematics_->getEEPositionInWorld(eeId, basePose, jointPosition);

        // we currently assume flat ground at height zero penetration is only z height
        Vector3s penetration;
        penetration << SCALAR(0.0), SCALAR(0.0), pos.z();

        return penetration;
    }

    /*!
	 * \brief Compute the endeffector force based on penetration and velocity
	 * @param eePenetration end-effector penetration
	 * @param eeVelocity end-effector velocity
	 * @return resulting force vecttor
	 */
    EEForceLinear computeEEForce(const Vector3s& eePenetration, const Velocity3S& eeVelocity)
    {
        EEForceLinear eeForce;

        computeDamperForce(eeForce, eePenetration, eeVelocity);

        smoothEEForce(eeForce, eePenetration);

        computeNormalSpring(eeForce, eePenetration(2) - zOffset_, eeVelocity.toImplementation()(2));

        return eeForce;
    }

    /*!
	 * \brief Smoothes out the endeffector forces
	 * @param eeForce endeffector force to modify
	 * @param eePenetration penetration of the surface
	 */
    void smoothEEForce(EEForceLinear& eeForce, const Vector3s& eePenetration)
    {
        switch (smoothing_)
        {
            case NONE:
                return;
            case SIGMOID:
                eeForce *= 1. / (1. + TRAIT::exp(eePenetration(2) * alpha_));
                return;
            case TANH:
                // same as sigmoid, maybe cheaper / more expensive to compute?
                eeForce *= 0.5 * TRAIT::tanh(-0.5 * eePenetration(2) * alpha_) + 0.5;
                return;
            case ABS:
                eeForce *= 0.5 * -eePenetration(2) * alpha_ / (1. + TRAIT::fabs(-eePenetration(2) * alpha_)) + 0.5;
                return;
            default:
                throw std::runtime_error("undefined smoothing function");
        }
    }

    /*!
	 * \brief computes the damper force \f$ \lambda = d \dot{x} \f$
	 * @param force force to be computed
	 * @param eePenetration endeffector penetration of the surface
	 * @param eeVelocity endeffector velocity
	 */
    void computeDamperForce(EEForceLinear& force, const Vector3s& eePenetration, const Velocity3S& eeVelocity)
    {
        force = -d_ * eeVelocity.toImplementation();
    }

    void computeNormalSpring(EEForceLinear& force, const SCALAR& p_N, const SCALAR& p_dot_N)
    {
        if (alpha_n_ > SCALAR(0))
        {
            force(2) += k_ * TRAIT::exp(-alpha_n_ * p_N);
        }
        else if (p_N <= SCALAR(0))
        {
            force(2) -= k_ * p_N;
        }
    }


    //! Instance of the kinematics
    std::shared_ptr<Kinematics> kinematics_;

    //! Type of velocity smoothing
    VELOCITY_SMOOTHING smoothing_;

    SCALAR k_;        //!< spring constant
    SCALAR d_;        //!< damper constant
    SCALAR alpha_;    //!< velocity smoothing coefficient
    SCALAR alpha_n_;  //!< normal force smoothing coefficient
    SCALAR zOffset_;  //!< vertical offset of the contact pane

    ActiveMap EEactive_;  //!< stores which endeffectors are active, i.e. can make contact
};
}  // namespace rbd
}  // namespace ct
