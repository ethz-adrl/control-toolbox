#ifndef IIT_ROBCOGEN__CT_INVERTEDPENDULUM_MISCELLANEOUS_H_
#define IIT_ROBCOGEN__CT_INVERTEDPENDULUM_MISCELLANEOUS_H_

#include "inertia_properties.h"
#include "transforms.h"

namespace iit {
namespace ct_InvertedPendulum {

/** \name Center of mass calculation
 * Computes the Center Of Mass (COM) position of the whole robot, in
 * base coordinates.
 *
 * Common parameters are the inertia properties of the robot and the set
 * of homogeneous coordinate transforms. If a joint status variable is
 * also passed, then the transforms are updated accordingly; otherwise,
 * they are not modified before being used.
 */
///@{
/**
 * \param inertia the inertia properties of the links of the robot
 * \param transforms the homogeneous coordinate transforms of the robot
 * \return the position of the Center Of Mass of the whole robot, expressed
 *         in base coordinates
 */
iit::rbd::Vector3d getWholeBodyCOM(
    const dyn::InertiaProperties& inertia,
    const HomogeneousTransforms& transforms);
/**
 * \param inertia the inertia properties of the links of the robot
 * \param q the joint status vector describing the configuration of the robot
 * \param transforms the homogeneous coordinate transforms of the robot
 * \return the position of the Center Of Mass of the whole robot, expressed
 *         in base coordinates
 */
iit::rbd::Vector3d getWholeBodyCOM(
    const dyn::InertiaProperties& inertia,
    const JointState& q,
    HomogeneousTransforms& transforms);
///@}

}
}

#endif
