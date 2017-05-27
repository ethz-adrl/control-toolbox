#ifndef IIT_ROBOT_CT_HYA_DECLARATIONS_H_
#define IIT_ROBOT_CT_HYA_DECLARATIONS_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace iit {
namespace ct_HyA {

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
    SAA = 0
    , SFE
    , HR
    , EFE
    , WR
    , WFE
};

enum LinkIdentifiers {
    HYABASE = 0
    , SHOULDER_AA
    , SHOULDER_FE
    , HUMERUS_R
    , ELBOW_FE
    , WRIST_R
    , WRIST_FE
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {SAA,SFE,HR,EFE,WR,WFE};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {HYABASE,SHOULDER_AA,SHOULDER_FE,HUMERUS_R,ELBOW_FE,WRIST_R,WRIST_FE};

}
}
#endif
