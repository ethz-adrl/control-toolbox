#ifndef IIT_ROBOT_HYQ_DECLARATIONS_H_
#define IIT_ROBOT_HYQ_DECLARATIONS_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace iit {
namespace HyQ {

static const int JointSpaceDimension = 12;
static const int jointsCount = 12;
/** The total number of rigid bodies of this robot, including the base */
static const int linksCount  = 13;

namespace tpl {
template <typename SCALAR>
using Column12d = Eigen::Matrix<SCALAR, 12, 1>;

template <typename SCALAR>
using JointState = Column12d<SCALAR>;
}

using Column12d = tpl::Column12d<double>;
typedef Column12d JointState;

enum JointIdentifiers {
    LF_HAA = 0
    , LF_HFE
    , LF_KFE
    , RF_HAA
    , RF_HFE
    , RF_KFE
    , LH_HAA
    , LH_HFE
    , LH_KFE
    , RH_HAA
    , RH_HFE
    , RH_KFE
};

enum LinkIdentifiers {
    TRUNK = 0
    , LF_HIPASSEMBLY
    , LF_UPPERLEG
    , LF_LOWERLEG
    , RF_HIPASSEMBLY
    , RF_UPPERLEG
    , RF_LOWERLEG
    , LH_HIPASSEMBLY
    , LH_UPPERLEG
    , LH_LOWERLEG
    , RH_HIPASSEMBLY
    , RH_UPPERLEG
    , RH_LOWERLEG
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {LF_HAA,LF_HFE,LF_KFE,RF_HAA,RF_HFE,RF_KFE,LH_HAA,LH_HFE,LH_KFE,RH_HAA,RH_HFE,RH_KFE};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {TRUNK,LF_HIPASSEMBLY,LF_UPPERLEG,LF_LOWERLEG,RF_HIPASSEMBLY,RF_UPPERLEG,RF_LOWERLEG,LH_HIPASSEMBLY,LH_UPPERLEG,LH_LOWERLEG,RH_HIPASSEMBLY,RH_UPPERLEG,RH_LOWERLEG};

}
}
#endif
