#ifndef IIT_ROBOT_CT_INVERTEDPENDULUM_DECLARATIONS_H_
#define IIT_ROBOT_CT_INVERTEDPENDULUM_DECLARATIONS_H_

#include <iit/rbd/rbd.h>

namespace iit {
namespace ct_InvertedPendulum {

static const int JointSpaceDimension = 1;
static const int jointsCount = 1;
/** The total number of rigid bodies of this robot, including the base */
static const int linksCount  = 2;

namespace tpl {
template <typename SCALAR>
using Column1d = iit::rbd::PlainMatrix<SCALAR, 1, 1>;

template <typename SCALAR>
using JointState = Column1d<SCALAR>;
}

using Column1d = tpl::Column1d<double>;
typedef Column1d JointState;

enum JointIdentifiers {
    JOINT1 = 0
};

enum LinkIdentifiers {
    INVERTEDPENDULUMBASE = 0
    , LINK1
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {JOINT1};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {INVERTEDPENDULUMBASE,LINK1};

}
}
#endif
