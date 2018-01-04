#ifndef CT_DOUBLEINVERTEDPENDULUM_JACOBIANS_H_
#define CT_DOUBLEINVERTEDPENDULUM_JACOBIANS_H_

		#include <iit/rbd/rbd.h>
#include <iit/rbd/TransformsBase.h>
#include <iit/rbd/traits/DoubleTrait.h>
#include "declarations.h"
#include "kinematics_parameters.h"

namespace iit {
namespace ct_DoubleInvertedPendulum {

template<typename SCALAR, int COLS, class M>
class JacobianT : public iit::rbd::JacobianBase<tpl::JointState<SCALAR>, COLS, M>
{};

namespace tpl {

/**
 *
 */
template <typename TRAIT>
class Jacobians {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef typename TRAIT::Scalar Scalar;
        typedef iit::rbd::Core<Scalar> CoreS;

        typedef JointState<Scalar> JState;

        class Type_fr_DoubleInvertedPendulumBase_J_fr_Link2 : public JacobianT<Scalar, 2, Type_fr_DoubleInvertedPendulumBase_J_fr_Link2>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_DoubleInvertedPendulumBase_J_fr_Link2();
            const Type_fr_DoubleInvertedPendulumBase_J_fr_Link2& update(const JState&);
        protected:
        };
        
        class Type_fr_DoubleInvertedPendulumBase_J_fr_Link1 : public JacobianT<Scalar, 1, Type_fr_DoubleInvertedPendulumBase_J_fr_Link1>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_DoubleInvertedPendulumBase_J_fr_Link1();
            const Type_fr_DoubleInvertedPendulumBase_J_fr_Link1& update(const JState&);
        protected:
        };
        
        class Type_fr_DoubleInvertedPendulumBase_J_fr_ee : public JacobianT<Scalar, 2, Type_fr_DoubleInvertedPendulumBase_J_fr_ee>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_DoubleInvertedPendulumBase_J_fr_ee();
            const Type_fr_DoubleInvertedPendulumBase_J_fr_ee& update(const JState&);
        protected:
        };
        
    public:
        Jacobians();
        void updateParameters();
    public:
        Type_fr_DoubleInvertedPendulumBase_J_fr_Link2 fr_DoubleInvertedPendulumBase_J_fr_Link2;
        Type_fr_DoubleInvertedPendulumBase_J_fr_Link1 fr_DoubleInvertedPendulumBase_J_fr_Link1;
        Type_fr_DoubleInvertedPendulumBase_J_fr_ee fr_DoubleInvertedPendulumBase_J_fr_ee;

    protected:

};

} //namespace tpl

using Jacobians = tpl::Jacobians<rbd::DoubleTrait>;

#include "jacobians.impl.h"


}
}

#endif
