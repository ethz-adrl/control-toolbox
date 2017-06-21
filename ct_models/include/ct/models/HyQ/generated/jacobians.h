#ifndef HYQ_JACOBIANS_H_
#define HYQ_JACOBIANS_H_

		#include <iit/rbd/rbd.h>
#include <iit/rbd/TransformsBase.h>
#include <iit/rbd/traits/DoubleTrait.h>
#include "declarations.h"
#include "kinematics_parameters.h"

namespace iit {
namespace HyQ {

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

        class Type_fr_trunk_J_LF_hipassemblyCOM : public JacobianT<Scalar, 1, Type_fr_trunk_J_LF_hipassemblyCOM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_trunk_J_LF_hipassemblyCOM();
            const Type_fr_trunk_J_LF_hipassemblyCOM& update(const JState&);
        protected:
        };
        
        class Type_fr_trunk_J_RF_hipassemblyCOM : public JacobianT<Scalar, 1, Type_fr_trunk_J_RF_hipassemblyCOM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_trunk_J_RF_hipassemblyCOM();
            const Type_fr_trunk_J_RF_hipassemblyCOM& update(const JState&);
        protected:
        };
        
        class Type_fr_trunk_J_LH_hipassemblyCOM : public JacobianT<Scalar, 1, Type_fr_trunk_J_LH_hipassemblyCOM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_trunk_J_LH_hipassemblyCOM();
            const Type_fr_trunk_J_LH_hipassemblyCOM& update(const JState&);
        protected:
        };
        
        class Type_fr_trunk_J_RH_hipassemblyCOM : public JacobianT<Scalar, 1, Type_fr_trunk_J_RH_hipassemblyCOM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_trunk_J_RH_hipassemblyCOM();
            const Type_fr_trunk_J_RH_hipassemblyCOM& update(const JState&);
        protected:
        };
        
        class Type_fr_trunk_J_LF_upperlegCOM : public JacobianT<Scalar, 2, Type_fr_trunk_J_LF_upperlegCOM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_trunk_J_LF_upperlegCOM();
            const Type_fr_trunk_J_LF_upperlegCOM& update(const JState&);
        protected:
        };
        
        class Type_fr_trunk_J_RF_upperlegCOM : public JacobianT<Scalar, 2, Type_fr_trunk_J_RF_upperlegCOM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_trunk_J_RF_upperlegCOM();
            const Type_fr_trunk_J_RF_upperlegCOM& update(const JState&);
        protected:
        };
        
        class Type_fr_trunk_J_LH_upperlegCOM : public JacobianT<Scalar, 2, Type_fr_trunk_J_LH_upperlegCOM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_trunk_J_LH_upperlegCOM();
            const Type_fr_trunk_J_LH_upperlegCOM& update(const JState&);
        protected:
        };
        
        class Type_fr_trunk_J_RH_upperlegCOM : public JacobianT<Scalar, 2, Type_fr_trunk_J_RH_upperlegCOM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_trunk_J_RH_upperlegCOM();
            const Type_fr_trunk_J_RH_upperlegCOM& update(const JState&);
        protected:
        };
        
        class Type_fr_trunk_J_LF_lowerlegCOM : public JacobianT<Scalar, 3, Type_fr_trunk_J_LF_lowerlegCOM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_trunk_J_LF_lowerlegCOM();
            const Type_fr_trunk_J_LF_lowerlegCOM& update(const JState&);
        protected:
        };
        
        class Type_fr_trunk_J_RF_lowerlegCOM : public JacobianT<Scalar, 3, Type_fr_trunk_J_RF_lowerlegCOM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_trunk_J_RF_lowerlegCOM();
            const Type_fr_trunk_J_RF_lowerlegCOM& update(const JState&);
        protected:
        };
        
        class Type_fr_trunk_J_LH_lowerlegCOM : public JacobianT<Scalar, 3, Type_fr_trunk_J_LH_lowerlegCOM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_trunk_J_LH_lowerlegCOM();
            const Type_fr_trunk_J_LH_lowerlegCOM& update(const JState&);
        protected:
        };
        
        class Type_fr_trunk_J_RH_lowerlegCOM : public JacobianT<Scalar, 3, Type_fr_trunk_J_RH_lowerlegCOM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_trunk_J_RH_lowerlegCOM();
            const Type_fr_trunk_J_RH_lowerlegCOM& update(const JState&);
        protected:
        };
        
        class Type_fr_trunk_J_fr_LF_foot : public JacobianT<Scalar, 3, Type_fr_trunk_J_fr_LF_foot>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_trunk_J_fr_LF_foot();
            const Type_fr_trunk_J_fr_LF_foot& update(const JState&);
        protected:
        };
        
        class Type_fr_trunk_J_fr_RF_foot : public JacobianT<Scalar, 3, Type_fr_trunk_J_fr_RF_foot>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_trunk_J_fr_RF_foot();
            const Type_fr_trunk_J_fr_RF_foot& update(const JState&);
        protected:
        };
        
        class Type_fr_trunk_J_fr_LH_foot : public JacobianT<Scalar, 3, Type_fr_trunk_J_fr_LH_foot>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_trunk_J_fr_LH_foot();
            const Type_fr_trunk_J_fr_LH_foot& update(const JState&);
        protected:
        };
        
        class Type_fr_trunk_J_fr_RH_foot : public JacobianT<Scalar, 3, Type_fr_trunk_J_fr_RH_foot>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_trunk_J_fr_RH_foot();
            const Type_fr_trunk_J_fr_RH_foot& update(const JState&);
        protected:
        };
        
    public:
        Jacobians();
        void updateParameters();
    public:
        Type_fr_trunk_J_LF_hipassemblyCOM fr_trunk_J_LF_hipassemblyCOM;
        Type_fr_trunk_J_RF_hipassemblyCOM fr_trunk_J_RF_hipassemblyCOM;
        Type_fr_trunk_J_LH_hipassemblyCOM fr_trunk_J_LH_hipassemblyCOM;
        Type_fr_trunk_J_RH_hipassemblyCOM fr_trunk_J_RH_hipassemblyCOM;
        Type_fr_trunk_J_LF_upperlegCOM fr_trunk_J_LF_upperlegCOM;
        Type_fr_trunk_J_RF_upperlegCOM fr_trunk_J_RF_upperlegCOM;
        Type_fr_trunk_J_LH_upperlegCOM fr_trunk_J_LH_upperlegCOM;
        Type_fr_trunk_J_RH_upperlegCOM fr_trunk_J_RH_upperlegCOM;
        Type_fr_trunk_J_LF_lowerlegCOM fr_trunk_J_LF_lowerlegCOM;
        Type_fr_trunk_J_RF_lowerlegCOM fr_trunk_J_RF_lowerlegCOM;
        Type_fr_trunk_J_LH_lowerlegCOM fr_trunk_J_LH_lowerlegCOM;
        Type_fr_trunk_J_RH_lowerlegCOM fr_trunk_J_RH_lowerlegCOM;
        Type_fr_trunk_J_fr_LF_foot fr_trunk_J_fr_LF_foot;
        Type_fr_trunk_J_fr_RF_foot fr_trunk_J_fr_RF_foot;
        Type_fr_trunk_J_fr_LH_foot fr_trunk_J_fr_LH_foot;
        Type_fr_trunk_J_fr_RH_foot fr_trunk_J_fr_RH_foot;

    protected:

};

} //namespace tpl

using Jacobians = tpl::Jacobians<rbd::DoubleTrait>;

#include "jacobians.impl.h"


}
}

#endif
