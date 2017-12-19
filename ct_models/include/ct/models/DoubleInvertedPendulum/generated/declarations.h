#ifndef IIT_ROBOT_CT_DOUBLEINVERTEDPENDULUM_DECLARATIONS_H_
#define IIT_ROBOT_CT_DOUBLEINVERTEDPENDULUM_DECLARATIONS_H_

#include <iit/rbd/rbd.h>

namespace iit {
namespace ct_DoubleInvertedPendulum {

static const int JointSpaceDimension = 2;
static const int jointsCount = 2;
/** The total number of rigid bodies of this robot, including the base */
static const int linksCount  = 3;

namespace tpl {
template <typename SCALAR>
using Column2d = iit::rbd::PlainMatrix<SCALAR, 2, 1>;

template <typename SCALAR>
using JointState = Column2d<SCALAR>;
}

using Column2d = tpl::Column2d<double>;
typedef Column2d JointState;

enum JointIdentifiers {
    JOINT1 = 0
    , JOINT2
};

enum LinkIdentifiers {
    DOUBLEINVERTEDPENDULUMBASE = 0
    , LINK1
    , LINK2
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {JOINT1,JOINT2};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {DOUBLEINVERTEDPENDULUMBASE,LINK1,LINK2};

}
}
#endif
