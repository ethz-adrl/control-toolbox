#ifndef TESTIRB4600_JACOBIANS_H_
#define TESTIRB4600_JACOBIANS_H_

#include <iit/rbd/TransformsBase.h>
#include <iit/rbd/traits/DoubleTrait.h>
#include "declarations.h"
#include "kinematics_parameters.h"

namespace iit {
namespace testirb4600 {

template<typename SCALAR, int COLS, class M>
class JacobianT : public iit::rbd::JacobianBase<tpl::JointState<SCALAR>, COLS, M>
{};

namespace tpl{

/**
 *
 */
template <typename TRAIT>
class Jacobians {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef typename TRAIT::Scalar SCALAR;

        typedef JointState<SCALAR> JState;

        class Type_fr_link0_J_fr_ee : public JacobianT<SCALAR, 6, Type_fr_link0_J_fr_ee>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_link0_J_fr_ee();
            const Type_fr_link0_J_fr_ee& update(const JState&);
        protected:
        };
        
    public:
        Jacobians();
        void updateParameters();
    public:
        Type_fr_link0_J_fr_ee fr_link0_J_fr_ee;

    protected:

};

} //namespace tpl

using Jacobians = tpl::Jacobians<rbd::DoubleTrait>;

#include "jacobians.impl.h"


}
}

#endif
