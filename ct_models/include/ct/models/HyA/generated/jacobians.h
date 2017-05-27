#ifndef CT_HYA_JACOBIANS_H_
#define CT_HYA_JACOBIANS_H_

#include <iit/rbd/TransformsBase.h>
#include <iit/rbd/traits/DoubleTrait.h>
#include "declarations.h"
#include "kinematics_parameters.h"

namespace iit {
namespace ct_HyA {

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

        class Type_fr_HyABase_J_fr_ee : public JacobianT<SCALAR, 6, Type_fr_HyABase_J_fr_ee>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_HyABase_J_fr_ee();
            const Type_fr_HyABase_J_fr_ee& update(const JState&);
        protected:
        };
        
        class Type_fr_HyABase_J_fr_Wrist_FE_COM : public JacobianT<SCALAR, 6, Type_fr_HyABase_J_fr_Wrist_FE_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_HyABase_J_fr_Wrist_FE_COM();
            const Type_fr_HyABase_J_fr_Wrist_FE_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_HyABase_J_fr_Wrist_FE : public JacobianT<SCALAR, 6, Type_fr_HyABase_J_fr_Wrist_FE>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_HyABase_J_fr_Wrist_FE();
            const Type_fr_HyABase_J_fr_Wrist_FE& update(const JState&);
        protected:
        };
        
        class Type_fr_HyABase_J_fr_Wrist_R_COM : public JacobianT<SCALAR, 5, Type_fr_HyABase_J_fr_Wrist_R_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_HyABase_J_fr_Wrist_R_COM();
            const Type_fr_HyABase_J_fr_Wrist_R_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_HyABase_J_fr_Wrist_R : public JacobianT<SCALAR, 5, Type_fr_HyABase_J_fr_Wrist_R>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_HyABase_J_fr_Wrist_R();
            const Type_fr_HyABase_J_fr_Wrist_R& update(const JState&);
        protected:
        };
        
        class Type_fr_HyABase_J_fr_Wrist_R_CTR : public JacobianT<SCALAR, 5, Type_fr_HyABase_J_fr_Wrist_R_CTR>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_HyABase_J_fr_Wrist_R_CTR();
            const Type_fr_HyABase_J_fr_Wrist_R_CTR& update(const JState&);
        protected:
        };
        
        class Type_fr_HyABase_J_fr_Elbow_FE_COM : public JacobianT<SCALAR, 4, Type_fr_HyABase_J_fr_Elbow_FE_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_HyABase_J_fr_Elbow_FE_COM();
            const Type_fr_HyABase_J_fr_Elbow_FE_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_HyABase_J_fr_Elbow_FE : public JacobianT<SCALAR, 4, Type_fr_HyABase_J_fr_Elbow_FE>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_HyABase_J_fr_Elbow_FE();
            const Type_fr_HyABase_J_fr_Elbow_FE& update(const JState&);
        protected:
        };
        
        class Type_fr_HyABase_J_fr_Elbow_FE_CTR : public JacobianT<SCALAR, 4, Type_fr_HyABase_J_fr_Elbow_FE_CTR>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_HyABase_J_fr_Elbow_FE_CTR();
            const Type_fr_HyABase_J_fr_Elbow_FE_CTR& update(const JState&);
        protected:
        };
        
        class Type_fr_HyABase_J_fr_Humerus_R_COM : public JacobianT<SCALAR, 3, Type_fr_HyABase_J_fr_Humerus_R_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_HyABase_J_fr_Humerus_R_COM();
            const Type_fr_HyABase_J_fr_Humerus_R_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_HyABase_J_fr_Humerus_R : public JacobianT<SCALAR, 3, Type_fr_HyABase_J_fr_Humerus_R>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_HyABase_J_fr_Humerus_R();
            const Type_fr_HyABase_J_fr_Humerus_R& update(const JState&);
        protected:
        };
        
        class Type_fr_HyABase_J_fr_Humerus_R_CTR : public JacobianT<SCALAR, 3, Type_fr_HyABase_J_fr_Humerus_R_CTR>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_HyABase_J_fr_Humerus_R_CTR();
            const Type_fr_HyABase_J_fr_Humerus_R_CTR& update(const JState&);
        protected:
        };
        
        class Type_fr_HyABase_J_fr_Shoulder_AA_CTR : public JacobianT<SCALAR, 1, Type_fr_HyABase_J_fr_Shoulder_AA_CTR>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_HyABase_J_fr_Shoulder_AA_CTR();
            const Type_fr_HyABase_J_fr_Shoulder_AA_CTR& update(const JState&);
        protected:
        };
        
        class Type_fr_HyABase_J_fr_Shoulder_FE_CTR : public JacobianT<SCALAR, 2, Type_fr_HyABase_J_fr_Shoulder_FE_CTR>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_HyABase_J_fr_Shoulder_FE_CTR();
            const Type_fr_HyABase_J_fr_Shoulder_FE_CTR& update(const JState&);
        protected:
        };
        
    public:
        Jacobians();
        void updateParameters();
    public:
        Type_fr_HyABase_J_fr_ee fr_HyABase_J_fr_ee;
        Type_fr_HyABase_J_fr_Wrist_FE_COM fr_HyABase_J_fr_Wrist_FE_COM;
        Type_fr_HyABase_J_fr_Wrist_FE fr_HyABase_J_fr_Wrist_FE;
        Type_fr_HyABase_J_fr_Wrist_R_COM fr_HyABase_J_fr_Wrist_R_COM;
        Type_fr_HyABase_J_fr_Wrist_R fr_HyABase_J_fr_Wrist_R;
        Type_fr_HyABase_J_fr_Wrist_R_CTR fr_HyABase_J_fr_Wrist_R_CTR;
        Type_fr_HyABase_J_fr_Elbow_FE_COM fr_HyABase_J_fr_Elbow_FE_COM;
        Type_fr_HyABase_J_fr_Elbow_FE fr_HyABase_J_fr_Elbow_FE;
        Type_fr_HyABase_J_fr_Elbow_FE_CTR fr_HyABase_J_fr_Elbow_FE_CTR;
        Type_fr_HyABase_J_fr_Humerus_R_COM fr_HyABase_J_fr_Humerus_R_COM;
        Type_fr_HyABase_J_fr_Humerus_R fr_HyABase_J_fr_Humerus_R;
        Type_fr_HyABase_J_fr_Humerus_R_CTR fr_HyABase_J_fr_Humerus_R_CTR;
        Type_fr_HyABase_J_fr_Shoulder_AA_CTR fr_HyABase_J_fr_Shoulder_AA_CTR;
        Type_fr_HyABase_J_fr_Shoulder_FE_CTR fr_HyABase_J_fr_Shoulder_FE_CTR;

    protected:

};

} //namespace tpl

using Jacobians = tpl::Jacobians<rbd::DoubleTrait>;

#include "jacobians.impl.h"


}
}

#endif
