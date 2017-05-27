#ifndef IIT_ROBOT_TESTIRB4600_DECLARATIONS_H_
#define IIT_ROBOT_TESTIRB4600_DECLARATIONS_H_

#include <Eigen/Dense>

namespace iit {
namespace testirb4600 {

static const int JointSpaceDimension = 6;
static const int jointsCount = 6;
/** The total number of rigid bodies of this robot, including the base */
static const int linksCount  = 7;

namespace tpl {
template <typename SCALAR>
using Column6d = Eigen::Matrix<SCALAR, 6, 1>;

template <typename SCALAR>
using JointState = Column6d<SCALAR>;
}

using Column6d = tpl::Column6d<double>;
typedef Column6d JointState;

enum JointIdentifiers {
    JA = 0
    , JB
    , JC
    , JD
    , JE
    , JF
};

enum LinkIdentifiers {
    LINK0 = 0
    , LINK1
    , LINK2
    , LINK3
    , LINK4
    , LINK5
    , LINK6
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {JA,JB,JC,JD,JE,JF};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {LINK0,LINK1,LINK2,LINK3,LINK4,LINK5,LINK6};

}
}
#endif
