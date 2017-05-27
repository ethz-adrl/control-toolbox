#ifndef IIT_ROBOT_CT_QUADROTOR_DECLARATIONS_H_
#define IIT_ROBOT_CT_QUADROTOR_DECLARATIONS_H_

#include <Eigen/Dense>

namespace iit {
namespace ct_quadrotor {

static const int JointSpaceDimension = 2;
static const int jointsCount = 2;
/** The total number of rigid bodies of this robot, including the base */
static const int linksCount  = 3;

namespace tpl {
template <typename SCALAR>
using Column2d = Eigen::Matrix<SCALAR, 2, 1>;

template <typename SCALAR>
using JointState = Column2d<SCALAR>;
}

using Column2d = tpl::Column2d<double>;
typedef Column2d JointState;

enum JointIdentifiers {
    JA = 0
    , JB
};

enum LinkIdentifiers {
    BODY = 0
    , LINK1
    , LINK2
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {JA,JB};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {BODY,LINK1,LINK2};

}
}
#endif
