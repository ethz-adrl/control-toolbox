#ifndef CT_HYA_TRANSFORMS_H_
#define CT_HYA_TRANSFORMS_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include <iit/rbd/traits/DoubleTrait.h>
#include "kinematics_parameters.h"

namespace iit {
namespace ct_HyA {

template<typename SCALAR, class M>
class TransformMotion : public iit::rbd::SpatialTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename SCALAR, class M>
class TransformForce : public iit::rbd::SpatialTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename SCALAR, class M>
class TransformHomogeneous : public iit::rbd::HomogeneousTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

namespace tpl {


/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
template <typename TRAIT>
class MotionTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar SCALAR;

    typedef JointState<SCALAR> JState;
    class Dummy {};
    typedef typename TransformMotion<SCALAR, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_HyABase_X_fr_Shoulder_AA : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_Shoulder_AA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Shoulder_AA();
        const Type_fr_HyABase_X_fr_Shoulder_AA& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Shoulder_FE : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_Shoulder_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Shoulder_FE();
        const Type_fr_HyABase_X_fr_Shoulder_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Humerus_R : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_Humerus_R>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Humerus_R();
        const Type_fr_HyABase_X_fr_Humerus_R& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Elbow_FE : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_Elbow_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Elbow_FE();
        const Type_fr_HyABase_X_fr_Elbow_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Wrist_R : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_Wrist_R>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Wrist_R();
        const Type_fr_HyABase_X_fr_Wrist_R& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Wrist_FE : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_Wrist_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Wrist_FE();
        const Type_fr_HyABase_X_fr_Wrist_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_ee : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_ee();
        const Type_fr_HyABase_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Shoulder_AA_COM : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_Shoulder_AA_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Shoulder_AA_COM();
        const Type_fr_HyABase_X_fr_Shoulder_AA_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Shoulder_FE_COM : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_Shoulder_FE_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Shoulder_FE_COM();
        const Type_fr_HyABase_X_fr_Shoulder_FE_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Humerus_R_COM : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_Humerus_R_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Humerus_R_COM();
        const Type_fr_HyABase_X_fr_Humerus_R_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Elbow_FE_COM : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_Elbow_FE_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Elbow_FE_COM();
        const Type_fr_HyABase_X_fr_Elbow_FE_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Wrist_R_COM : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_Wrist_R_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Wrist_R_COM();
        const Type_fr_HyABase_X_fr_Wrist_R_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Wrist_FE_COM : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_Wrist_FE_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Wrist_FE_COM();
        const Type_fr_HyABase_X_fr_Wrist_FE_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Shoulder_AA_CTR : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_Shoulder_AA_CTR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Shoulder_AA_CTR();
        const Type_fr_HyABase_X_fr_Shoulder_AA_CTR& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Shoulder_FE_CTR : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_Shoulder_FE_CTR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Shoulder_FE_CTR();
        const Type_fr_HyABase_X_fr_Shoulder_FE_CTR& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Humerus_R_CTR : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_Humerus_R_CTR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Humerus_R_CTR();
        const Type_fr_HyABase_X_fr_Humerus_R_CTR& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Elbow_FE_CTR : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_Elbow_FE_CTR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Elbow_FE_CTR();
        const Type_fr_HyABase_X_fr_Elbow_FE_CTR& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Wrist_R_CTR : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_Wrist_R_CTR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Wrist_R_CTR();
        const Type_fr_HyABase_X_fr_Wrist_R_CTR& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_FE_X_fr_Wrist_FE : public TransformMotion<SCALAR, Type_fr_Shoulder_FE_X_fr_Wrist_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_FE_X_fr_Wrist_FE();
        const Type_fr_Shoulder_FE_X_fr_Wrist_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Humerus_R_X_fr_Wrist_FE : public TransformMotion<SCALAR, Type_fr_Humerus_R_X_fr_Wrist_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Humerus_R_X_fr_Wrist_FE();
        const Type_fr_Humerus_R_X_fr_Wrist_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Elbow_FE_X_fr_Wrist_FE : public TransformMotion<SCALAR, Type_fr_Elbow_FE_X_fr_Wrist_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Elbow_FE_X_fr_Wrist_FE();
        const Type_fr_Elbow_FE_X_fr_Wrist_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_R_X_fr_Wrist_FE : public TransformMotion<SCALAR, Type_fr_Wrist_R_X_fr_Wrist_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_R_X_fr_Wrist_FE();
        const Type_fr_Wrist_R_X_fr_Wrist_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_FE_X_fr_Shoulder_FE : public TransformMotion<SCALAR, Type_fr_Wrist_FE_X_fr_Shoulder_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_FE_X_fr_Shoulder_FE();
        const Type_fr_Wrist_FE_X_fr_Shoulder_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_FE_X_fr_ee : public TransformMotion<SCALAR, Type_fr_Shoulder_FE_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_FE_X_fr_ee();
        const Type_fr_Shoulder_FE_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_Humerus_R_X_fr_ee : public TransformMotion<SCALAR, Type_fr_Humerus_R_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Humerus_R_X_fr_ee();
        const Type_fr_Humerus_R_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_Elbow_FE_X_fr_ee : public TransformMotion<SCALAR, Type_fr_Elbow_FE_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Elbow_FE_X_fr_ee();
        const Type_fr_Elbow_FE_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_R_X_fr_ee : public TransformMotion<SCALAR, Type_fr_Wrist_R_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_R_X_fr_ee();
        const Type_fr_Wrist_R_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_FE_X_fr_ee : public TransformMotion<SCALAR, Type_fr_Wrist_FE_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_FE_X_fr_ee();
        const Type_fr_Wrist_FE_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_AA_X_fr_HyABase : public TransformMotion<SCALAR, Type_fr_Shoulder_AA_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_AA_X_fr_HyABase();
        const Type_fr_Shoulder_AA_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_FE_X_fr_HyABase : public TransformMotion<SCALAR, Type_fr_Shoulder_FE_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_FE_X_fr_HyABase();
        const Type_fr_Shoulder_FE_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_Humerus_R_X_fr_HyABase : public TransformMotion<SCALAR, Type_fr_Humerus_R_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Humerus_R_X_fr_HyABase();
        const Type_fr_Humerus_R_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_Elbow_FE_X_fr_HyABase : public TransformMotion<SCALAR, Type_fr_Elbow_FE_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Elbow_FE_X_fr_HyABase();
        const Type_fr_Elbow_FE_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_R_X_fr_HyABase : public TransformMotion<SCALAR, Type_fr_Wrist_R_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_R_X_fr_HyABase();
        const Type_fr_Wrist_R_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_FE_X_fr_HyABase : public TransformMotion<SCALAR, Type_fr_Wrist_FE_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_FE_X_fr_HyABase();
        const Type_fr_Wrist_FE_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_ee_X_fr_HyABase : public TransformMotion<SCALAR, Type_fr_ee_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ee_X_fr_HyABase();
        const Type_fr_ee_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_SAA : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_SAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_SAA();
        const Type_fr_HyABase_X_fr_SAA& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_SFE : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_SFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_SFE();
        const Type_fr_HyABase_X_fr_SFE& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_HR : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_HR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_HR();
        const Type_fr_HyABase_X_fr_HR& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_EFE : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_EFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_EFE();
        const Type_fr_HyABase_X_fr_EFE& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_WR : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_WR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_WR();
        const Type_fr_HyABase_X_fr_WR& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_WFE : public TransformMotion<SCALAR, Type_fr_HyABase_X_fr_WFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_WFE();
        const Type_fr_HyABase_X_fr_WFE& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_FE_X_fr_Shoulder_AA : public TransformMotion<SCALAR, Type_fr_Shoulder_FE_X_fr_Shoulder_AA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_FE_X_fr_Shoulder_AA();
        const Type_fr_Shoulder_FE_X_fr_Shoulder_AA& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_AA_X_fr_Shoulder_FE : public TransformMotion<SCALAR, Type_fr_Shoulder_AA_X_fr_Shoulder_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_AA_X_fr_Shoulder_FE();
        const Type_fr_Shoulder_AA_X_fr_Shoulder_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Humerus_R_X_fr_Shoulder_FE : public TransformMotion<SCALAR, Type_fr_Humerus_R_X_fr_Shoulder_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Humerus_R_X_fr_Shoulder_FE();
        const Type_fr_Humerus_R_X_fr_Shoulder_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_FE_X_fr_Humerus_R : public TransformMotion<SCALAR, Type_fr_Shoulder_FE_X_fr_Humerus_R>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_FE_X_fr_Humerus_R();
        const Type_fr_Shoulder_FE_X_fr_Humerus_R& update(const JState&);
    protected:
    };
    
    class Type_fr_Elbow_FE_X_fr_Humerus_R : public TransformMotion<SCALAR, Type_fr_Elbow_FE_X_fr_Humerus_R>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Elbow_FE_X_fr_Humerus_R();
        const Type_fr_Elbow_FE_X_fr_Humerus_R& update(const JState&);
    protected:
    };
    
    class Type_fr_Humerus_R_X_fr_Elbow_FE : public TransformMotion<SCALAR, Type_fr_Humerus_R_X_fr_Elbow_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Humerus_R_X_fr_Elbow_FE();
        const Type_fr_Humerus_R_X_fr_Elbow_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_R_X_fr_Elbow_FE : public TransformMotion<SCALAR, Type_fr_Wrist_R_X_fr_Elbow_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_R_X_fr_Elbow_FE();
        const Type_fr_Wrist_R_X_fr_Elbow_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Elbow_FE_X_fr_Wrist_R : public TransformMotion<SCALAR, Type_fr_Elbow_FE_X_fr_Wrist_R>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Elbow_FE_X_fr_Wrist_R();
        const Type_fr_Elbow_FE_X_fr_Wrist_R& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_FE_X_fr_Wrist_R : public TransformMotion<SCALAR, Type_fr_Wrist_FE_X_fr_Wrist_R>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_FE_X_fr_Wrist_R();
        const Type_fr_Wrist_FE_X_fr_Wrist_R& update(const JState&);
    protected:
    };
    
public:
    MotionTransforms();
    void updateParameters();
    Type_fr_HyABase_X_fr_Shoulder_AA fr_HyABase_X_fr_Shoulder_AA;
    Type_fr_HyABase_X_fr_Shoulder_FE fr_HyABase_X_fr_Shoulder_FE;
    Type_fr_HyABase_X_fr_Humerus_R fr_HyABase_X_fr_Humerus_R;
    Type_fr_HyABase_X_fr_Elbow_FE fr_HyABase_X_fr_Elbow_FE;
    Type_fr_HyABase_X_fr_Wrist_R fr_HyABase_X_fr_Wrist_R;
    Type_fr_HyABase_X_fr_Wrist_FE fr_HyABase_X_fr_Wrist_FE;
    Type_fr_HyABase_X_fr_ee fr_HyABase_X_fr_ee;
    Type_fr_HyABase_X_fr_Shoulder_AA_COM fr_HyABase_X_fr_Shoulder_AA_COM;
    Type_fr_HyABase_X_fr_Shoulder_FE_COM fr_HyABase_X_fr_Shoulder_FE_COM;
    Type_fr_HyABase_X_fr_Humerus_R_COM fr_HyABase_X_fr_Humerus_R_COM;
    Type_fr_HyABase_X_fr_Elbow_FE_COM fr_HyABase_X_fr_Elbow_FE_COM;
    Type_fr_HyABase_X_fr_Wrist_R_COM fr_HyABase_X_fr_Wrist_R_COM;
    Type_fr_HyABase_X_fr_Wrist_FE_COM fr_HyABase_X_fr_Wrist_FE_COM;
    Type_fr_HyABase_X_fr_Shoulder_AA_CTR fr_HyABase_X_fr_Shoulder_AA_CTR;
    Type_fr_HyABase_X_fr_Shoulder_FE_CTR fr_HyABase_X_fr_Shoulder_FE_CTR;
    Type_fr_HyABase_X_fr_Humerus_R_CTR fr_HyABase_X_fr_Humerus_R_CTR;
    Type_fr_HyABase_X_fr_Elbow_FE_CTR fr_HyABase_X_fr_Elbow_FE_CTR;
    Type_fr_HyABase_X_fr_Wrist_R_CTR fr_HyABase_X_fr_Wrist_R_CTR;
    Type_fr_Shoulder_FE_X_fr_Wrist_FE fr_Shoulder_FE_X_fr_Wrist_FE;
    Type_fr_Humerus_R_X_fr_Wrist_FE fr_Humerus_R_X_fr_Wrist_FE;
    Type_fr_Elbow_FE_X_fr_Wrist_FE fr_Elbow_FE_X_fr_Wrist_FE;
    Type_fr_Wrist_R_X_fr_Wrist_FE fr_Wrist_R_X_fr_Wrist_FE;
    Type_fr_Wrist_FE_X_fr_Shoulder_FE fr_Wrist_FE_X_fr_Shoulder_FE;
    Type_fr_Shoulder_FE_X_fr_ee fr_Shoulder_FE_X_fr_ee;
    Type_fr_Humerus_R_X_fr_ee fr_Humerus_R_X_fr_ee;
    Type_fr_Elbow_FE_X_fr_ee fr_Elbow_FE_X_fr_ee;
    Type_fr_Wrist_R_X_fr_ee fr_Wrist_R_X_fr_ee;
    Type_fr_Wrist_FE_X_fr_ee fr_Wrist_FE_X_fr_ee;
    Type_fr_Shoulder_AA_X_fr_HyABase fr_Shoulder_AA_X_fr_HyABase;
    Type_fr_Shoulder_FE_X_fr_HyABase fr_Shoulder_FE_X_fr_HyABase;
    Type_fr_Humerus_R_X_fr_HyABase fr_Humerus_R_X_fr_HyABase;
    Type_fr_Elbow_FE_X_fr_HyABase fr_Elbow_FE_X_fr_HyABase;
    Type_fr_Wrist_R_X_fr_HyABase fr_Wrist_R_X_fr_HyABase;
    Type_fr_Wrist_FE_X_fr_HyABase fr_Wrist_FE_X_fr_HyABase;
    Type_fr_ee_X_fr_HyABase fr_ee_X_fr_HyABase;
    Type_fr_HyABase_X_fr_SAA fr_HyABase_X_fr_SAA;
    Type_fr_HyABase_X_fr_SFE fr_HyABase_X_fr_SFE;
    Type_fr_HyABase_X_fr_HR fr_HyABase_X_fr_HR;
    Type_fr_HyABase_X_fr_EFE fr_HyABase_X_fr_EFE;
    Type_fr_HyABase_X_fr_WR fr_HyABase_X_fr_WR;
    Type_fr_HyABase_X_fr_WFE fr_HyABase_X_fr_WFE;
    Type_fr_Shoulder_FE_X_fr_Shoulder_AA fr_Shoulder_FE_X_fr_Shoulder_AA;
    Type_fr_Shoulder_AA_X_fr_Shoulder_FE fr_Shoulder_AA_X_fr_Shoulder_FE;
    Type_fr_Humerus_R_X_fr_Shoulder_FE fr_Humerus_R_X_fr_Shoulder_FE;
    Type_fr_Shoulder_FE_X_fr_Humerus_R fr_Shoulder_FE_X_fr_Humerus_R;
    Type_fr_Elbow_FE_X_fr_Humerus_R fr_Elbow_FE_X_fr_Humerus_R;
    Type_fr_Humerus_R_X_fr_Elbow_FE fr_Humerus_R_X_fr_Elbow_FE;
    Type_fr_Wrist_R_X_fr_Elbow_FE fr_Wrist_R_X_fr_Elbow_FE;
    Type_fr_Elbow_FE_X_fr_Wrist_R fr_Elbow_FE_X_fr_Wrist_R;
    Type_fr_Wrist_FE_X_fr_Wrist_R fr_Wrist_FE_X_fr_Wrist_R;

protected:

}; //class 'MotionTransforms'

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial force vectors.
 */
template <typename TRAIT>
class ForceTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar SCALAR;

    typedef JointState<SCALAR> JState;
    class Dummy {};
    typedef typename TransformForce<SCALAR, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_HyABase_X_fr_Shoulder_AA : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_Shoulder_AA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Shoulder_AA();
        const Type_fr_HyABase_X_fr_Shoulder_AA& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Shoulder_FE : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_Shoulder_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Shoulder_FE();
        const Type_fr_HyABase_X_fr_Shoulder_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Humerus_R : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_Humerus_R>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Humerus_R();
        const Type_fr_HyABase_X_fr_Humerus_R& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Elbow_FE : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_Elbow_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Elbow_FE();
        const Type_fr_HyABase_X_fr_Elbow_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Wrist_R : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_Wrist_R>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Wrist_R();
        const Type_fr_HyABase_X_fr_Wrist_R& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Wrist_FE : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_Wrist_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Wrist_FE();
        const Type_fr_HyABase_X_fr_Wrist_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_ee : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_ee();
        const Type_fr_HyABase_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Shoulder_AA_COM : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_Shoulder_AA_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Shoulder_AA_COM();
        const Type_fr_HyABase_X_fr_Shoulder_AA_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Shoulder_FE_COM : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_Shoulder_FE_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Shoulder_FE_COM();
        const Type_fr_HyABase_X_fr_Shoulder_FE_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Humerus_R_COM : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_Humerus_R_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Humerus_R_COM();
        const Type_fr_HyABase_X_fr_Humerus_R_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Elbow_FE_COM : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_Elbow_FE_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Elbow_FE_COM();
        const Type_fr_HyABase_X_fr_Elbow_FE_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Wrist_R_COM : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_Wrist_R_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Wrist_R_COM();
        const Type_fr_HyABase_X_fr_Wrist_R_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Wrist_FE_COM : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_Wrist_FE_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Wrist_FE_COM();
        const Type_fr_HyABase_X_fr_Wrist_FE_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Shoulder_AA_CTR : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_Shoulder_AA_CTR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Shoulder_AA_CTR();
        const Type_fr_HyABase_X_fr_Shoulder_AA_CTR& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Shoulder_FE_CTR : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_Shoulder_FE_CTR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Shoulder_FE_CTR();
        const Type_fr_HyABase_X_fr_Shoulder_FE_CTR& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Humerus_R_CTR : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_Humerus_R_CTR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Humerus_R_CTR();
        const Type_fr_HyABase_X_fr_Humerus_R_CTR& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Elbow_FE_CTR : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_Elbow_FE_CTR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Elbow_FE_CTR();
        const Type_fr_HyABase_X_fr_Elbow_FE_CTR& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Wrist_R_CTR : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_Wrist_R_CTR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Wrist_R_CTR();
        const Type_fr_HyABase_X_fr_Wrist_R_CTR& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_FE_X_fr_Wrist_FE : public TransformForce<SCALAR, Type_fr_Shoulder_FE_X_fr_Wrist_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_FE_X_fr_Wrist_FE();
        const Type_fr_Shoulder_FE_X_fr_Wrist_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Humerus_R_X_fr_Wrist_FE : public TransformForce<SCALAR, Type_fr_Humerus_R_X_fr_Wrist_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Humerus_R_X_fr_Wrist_FE();
        const Type_fr_Humerus_R_X_fr_Wrist_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Elbow_FE_X_fr_Wrist_FE : public TransformForce<SCALAR, Type_fr_Elbow_FE_X_fr_Wrist_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Elbow_FE_X_fr_Wrist_FE();
        const Type_fr_Elbow_FE_X_fr_Wrist_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_R_X_fr_Wrist_FE : public TransformForce<SCALAR, Type_fr_Wrist_R_X_fr_Wrist_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_R_X_fr_Wrist_FE();
        const Type_fr_Wrist_R_X_fr_Wrist_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_FE_X_fr_Shoulder_FE : public TransformForce<SCALAR, Type_fr_Wrist_FE_X_fr_Shoulder_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_FE_X_fr_Shoulder_FE();
        const Type_fr_Wrist_FE_X_fr_Shoulder_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_FE_X_fr_ee : public TransformForce<SCALAR, Type_fr_Shoulder_FE_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_FE_X_fr_ee();
        const Type_fr_Shoulder_FE_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_Humerus_R_X_fr_ee : public TransformForce<SCALAR, Type_fr_Humerus_R_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Humerus_R_X_fr_ee();
        const Type_fr_Humerus_R_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_Elbow_FE_X_fr_ee : public TransformForce<SCALAR, Type_fr_Elbow_FE_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Elbow_FE_X_fr_ee();
        const Type_fr_Elbow_FE_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_R_X_fr_ee : public TransformForce<SCALAR, Type_fr_Wrist_R_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_R_X_fr_ee();
        const Type_fr_Wrist_R_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_FE_X_fr_ee : public TransformForce<SCALAR, Type_fr_Wrist_FE_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_FE_X_fr_ee();
        const Type_fr_Wrist_FE_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_AA_X_fr_HyABase : public TransformForce<SCALAR, Type_fr_Shoulder_AA_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_AA_X_fr_HyABase();
        const Type_fr_Shoulder_AA_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_FE_X_fr_HyABase : public TransformForce<SCALAR, Type_fr_Shoulder_FE_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_FE_X_fr_HyABase();
        const Type_fr_Shoulder_FE_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_Humerus_R_X_fr_HyABase : public TransformForce<SCALAR, Type_fr_Humerus_R_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Humerus_R_X_fr_HyABase();
        const Type_fr_Humerus_R_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_Elbow_FE_X_fr_HyABase : public TransformForce<SCALAR, Type_fr_Elbow_FE_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Elbow_FE_X_fr_HyABase();
        const Type_fr_Elbow_FE_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_R_X_fr_HyABase : public TransformForce<SCALAR, Type_fr_Wrist_R_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_R_X_fr_HyABase();
        const Type_fr_Wrist_R_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_FE_X_fr_HyABase : public TransformForce<SCALAR, Type_fr_Wrist_FE_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_FE_X_fr_HyABase();
        const Type_fr_Wrist_FE_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_ee_X_fr_HyABase : public TransformForce<SCALAR, Type_fr_ee_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ee_X_fr_HyABase();
        const Type_fr_ee_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_SAA : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_SAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_SAA();
        const Type_fr_HyABase_X_fr_SAA& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_SFE : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_SFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_SFE();
        const Type_fr_HyABase_X_fr_SFE& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_HR : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_HR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_HR();
        const Type_fr_HyABase_X_fr_HR& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_EFE : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_EFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_EFE();
        const Type_fr_HyABase_X_fr_EFE& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_WR : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_WR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_WR();
        const Type_fr_HyABase_X_fr_WR& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_WFE : public TransformForce<SCALAR, Type_fr_HyABase_X_fr_WFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_WFE();
        const Type_fr_HyABase_X_fr_WFE& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_FE_X_fr_Shoulder_AA : public TransformForce<SCALAR, Type_fr_Shoulder_FE_X_fr_Shoulder_AA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_FE_X_fr_Shoulder_AA();
        const Type_fr_Shoulder_FE_X_fr_Shoulder_AA& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_AA_X_fr_Shoulder_FE : public TransformForce<SCALAR, Type_fr_Shoulder_AA_X_fr_Shoulder_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_AA_X_fr_Shoulder_FE();
        const Type_fr_Shoulder_AA_X_fr_Shoulder_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Humerus_R_X_fr_Shoulder_FE : public TransformForce<SCALAR, Type_fr_Humerus_R_X_fr_Shoulder_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Humerus_R_X_fr_Shoulder_FE();
        const Type_fr_Humerus_R_X_fr_Shoulder_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_FE_X_fr_Humerus_R : public TransformForce<SCALAR, Type_fr_Shoulder_FE_X_fr_Humerus_R>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_FE_X_fr_Humerus_R();
        const Type_fr_Shoulder_FE_X_fr_Humerus_R& update(const JState&);
    protected:
    };
    
    class Type_fr_Elbow_FE_X_fr_Humerus_R : public TransformForce<SCALAR, Type_fr_Elbow_FE_X_fr_Humerus_R>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Elbow_FE_X_fr_Humerus_R();
        const Type_fr_Elbow_FE_X_fr_Humerus_R& update(const JState&);
    protected:
    };
    
    class Type_fr_Humerus_R_X_fr_Elbow_FE : public TransformForce<SCALAR, Type_fr_Humerus_R_X_fr_Elbow_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Humerus_R_X_fr_Elbow_FE();
        const Type_fr_Humerus_R_X_fr_Elbow_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_R_X_fr_Elbow_FE : public TransformForce<SCALAR, Type_fr_Wrist_R_X_fr_Elbow_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_R_X_fr_Elbow_FE();
        const Type_fr_Wrist_R_X_fr_Elbow_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Elbow_FE_X_fr_Wrist_R : public TransformForce<SCALAR, Type_fr_Elbow_FE_X_fr_Wrist_R>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Elbow_FE_X_fr_Wrist_R();
        const Type_fr_Elbow_FE_X_fr_Wrist_R& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_FE_X_fr_Wrist_R : public TransformForce<SCALAR, Type_fr_Wrist_FE_X_fr_Wrist_R>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_FE_X_fr_Wrist_R();
        const Type_fr_Wrist_FE_X_fr_Wrist_R& update(const JState&);
    protected:
    };
    
public:
    ForceTransforms();
    void updateParameters();
    Type_fr_HyABase_X_fr_Shoulder_AA fr_HyABase_X_fr_Shoulder_AA;
    Type_fr_HyABase_X_fr_Shoulder_FE fr_HyABase_X_fr_Shoulder_FE;
    Type_fr_HyABase_X_fr_Humerus_R fr_HyABase_X_fr_Humerus_R;
    Type_fr_HyABase_X_fr_Elbow_FE fr_HyABase_X_fr_Elbow_FE;
    Type_fr_HyABase_X_fr_Wrist_R fr_HyABase_X_fr_Wrist_R;
    Type_fr_HyABase_X_fr_Wrist_FE fr_HyABase_X_fr_Wrist_FE;
    Type_fr_HyABase_X_fr_ee fr_HyABase_X_fr_ee;
    Type_fr_HyABase_X_fr_Shoulder_AA_COM fr_HyABase_X_fr_Shoulder_AA_COM;
    Type_fr_HyABase_X_fr_Shoulder_FE_COM fr_HyABase_X_fr_Shoulder_FE_COM;
    Type_fr_HyABase_X_fr_Humerus_R_COM fr_HyABase_X_fr_Humerus_R_COM;
    Type_fr_HyABase_X_fr_Elbow_FE_COM fr_HyABase_X_fr_Elbow_FE_COM;
    Type_fr_HyABase_X_fr_Wrist_R_COM fr_HyABase_X_fr_Wrist_R_COM;
    Type_fr_HyABase_X_fr_Wrist_FE_COM fr_HyABase_X_fr_Wrist_FE_COM;
    Type_fr_HyABase_X_fr_Shoulder_AA_CTR fr_HyABase_X_fr_Shoulder_AA_CTR;
    Type_fr_HyABase_X_fr_Shoulder_FE_CTR fr_HyABase_X_fr_Shoulder_FE_CTR;
    Type_fr_HyABase_X_fr_Humerus_R_CTR fr_HyABase_X_fr_Humerus_R_CTR;
    Type_fr_HyABase_X_fr_Elbow_FE_CTR fr_HyABase_X_fr_Elbow_FE_CTR;
    Type_fr_HyABase_X_fr_Wrist_R_CTR fr_HyABase_X_fr_Wrist_R_CTR;
    Type_fr_Shoulder_FE_X_fr_Wrist_FE fr_Shoulder_FE_X_fr_Wrist_FE;
    Type_fr_Humerus_R_X_fr_Wrist_FE fr_Humerus_R_X_fr_Wrist_FE;
    Type_fr_Elbow_FE_X_fr_Wrist_FE fr_Elbow_FE_X_fr_Wrist_FE;
    Type_fr_Wrist_R_X_fr_Wrist_FE fr_Wrist_R_X_fr_Wrist_FE;
    Type_fr_Wrist_FE_X_fr_Shoulder_FE fr_Wrist_FE_X_fr_Shoulder_FE;
    Type_fr_Shoulder_FE_X_fr_ee fr_Shoulder_FE_X_fr_ee;
    Type_fr_Humerus_R_X_fr_ee fr_Humerus_R_X_fr_ee;
    Type_fr_Elbow_FE_X_fr_ee fr_Elbow_FE_X_fr_ee;
    Type_fr_Wrist_R_X_fr_ee fr_Wrist_R_X_fr_ee;
    Type_fr_Wrist_FE_X_fr_ee fr_Wrist_FE_X_fr_ee;
    Type_fr_Shoulder_AA_X_fr_HyABase fr_Shoulder_AA_X_fr_HyABase;
    Type_fr_Shoulder_FE_X_fr_HyABase fr_Shoulder_FE_X_fr_HyABase;
    Type_fr_Humerus_R_X_fr_HyABase fr_Humerus_R_X_fr_HyABase;
    Type_fr_Elbow_FE_X_fr_HyABase fr_Elbow_FE_X_fr_HyABase;
    Type_fr_Wrist_R_X_fr_HyABase fr_Wrist_R_X_fr_HyABase;
    Type_fr_Wrist_FE_X_fr_HyABase fr_Wrist_FE_X_fr_HyABase;
    Type_fr_ee_X_fr_HyABase fr_ee_X_fr_HyABase;
    Type_fr_HyABase_X_fr_SAA fr_HyABase_X_fr_SAA;
    Type_fr_HyABase_X_fr_SFE fr_HyABase_X_fr_SFE;
    Type_fr_HyABase_X_fr_HR fr_HyABase_X_fr_HR;
    Type_fr_HyABase_X_fr_EFE fr_HyABase_X_fr_EFE;
    Type_fr_HyABase_X_fr_WR fr_HyABase_X_fr_WR;
    Type_fr_HyABase_X_fr_WFE fr_HyABase_X_fr_WFE;
    Type_fr_Shoulder_FE_X_fr_Shoulder_AA fr_Shoulder_FE_X_fr_Shoulder_AA;
    Type_fr_Shoulder_AA_X_fr_Shoulder_FE fr_Shoulder_AA_X_fr_Shoulder_FE;
    Type_fr_Humerus_R_X_fr_Shoulder_FE fr_Humerus_R_X_fr_Shoulder_FE;
    Type_fr_Shoulder_FE_X_fr_Humerus_R fr_Shoulder_FE_X_fr_Humerus_R;
    Type_fr_Elbow_FE_X_fr_Humerus_R fr_Elbow_FE_X_fr_Humerus_R;
    Type_fr_Humerus_R_X_fr_Elbow_FE fr_Humerus_R_X_fr_Elbow_FE;
    Type_fr_Wrist_R_X_fr_Elbow_FE fr_Wrist_R_X_fr_Elbow_FE;
    Type_fr_Elbow_FE_X_fr_Wrist_R fr_Elbow_FE_X_fr_Wrist_R;
    Type_fr_Wrist_FE_X_fr_Wrist_R fr_Wrist_FE_X_fr_Wrist_R;

protected:

}; //class 'ForceTransforms'

/**
 * The class with the homogeneous (4x4) coordinates transformation
 * matrices.
 */
template <typename TRAIT>
class HomogeneousTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar SCALAR;

    typedef JointState<SCALAR> JState;
    class Dummy {};
    typedef typename TransformHomogeneous<SCALAR, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_HyABase_X_fr_Shoulder_AA : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_Shoulder_AA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Shoulder_AA();
        const Type_fr_HyABase_X_fr_Shoulder_AA& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Shoulder_FE : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_Shoulder_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Shoulder_FE();
        const Type_fr_HyABase_X_fr_Shoulder_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Humerus_R : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_Humerus_R>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Humerus_R();
        const Type_fr_HyABase_X_fr_Humerus_R& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Elbow_FE : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_Elbow_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Elbow_FE();
        const Type_fr_HyABase_X_fr_Elbow_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Wrist_R : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_Wrist_R>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Wrist_R();
        const Type_fr_HyABase_X_fr_Wrist_R& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Wrist_FE : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_Wrist_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Wrist_FE();
        const Type_fr_HyABase_X_fr_Wrist_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_ee : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_ee();
        const Type_fr_HyABase_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Shoulder_AA_COM : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_Shoulder_AA_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Shoulder_AA_COM();
        const Type_fr_HyABase_X_fr_Shoulder_AA_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Shoulder_FE_COM : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_Shoulder_FE_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Shoulder_FE_COM();
        const Type_fr_HyABase_X_fr_Shoulder_FE_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Humerus_R_COM : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_Humerus_R_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Humerus_R_COM();
        const Type_fr_HyABase_X_fr_Humerus_R_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Elbow_FE_COM : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_Elbow_FE_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Elbow_FE_COM();
        const Type_fr_HyABase_X_fr_Elbow_FE_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Wrist_R_COM : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_Wrist_R_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Wrist_R_COM();
        const Type_fr_HyABase_X_fr_Wrist_R_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Wrist_FE_COM : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_Wrist_FE_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Wrist_FE_COM();
        const Type_fr_HyABase_X_fr_Wrist_FE_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Shoulder_AA_CTR : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_Shoulder_AA_CTR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Shoulder_AA_CTR();
        const Type_fr_HyABase_X_fr_Shoulder_AA_CTR& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Shoulder_FE_CTR : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_Shoulder_FE_CTR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Shoulder_FE_CTR();
        const Type_fr_HyABase_X_fr_Shoulder_FE_CTR& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Humerus_R_CTR : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_Humerus_R_CTR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Humerus_R_CTR();
        const Type_fr_HyABase_X_fr_Humerus_R_CTR& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Elbow_FE_CTR : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_Elbow_FE_CTR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Elbow_FE_CTR();
        const Type_fr_HyABase_X_fr_Elbow_FE_CTR& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_Wrist_R_CTR : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_Wrist_R_CTR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_Wrist_R_CTR();
        const Type_fr_HyABase_X_fr_Wrist_R_CTR& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_FE_X_fr_Wrist_FE : public TransformHomogeneous<SCALAR, Type_fr_Shoulder_FE_X_fr_Wrist_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_FE_X_fr_Wrist_FE();
        const Type_fr_Shoulder_FE_X_fr_Wrist_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Humerus_R_X_fr_Wrist_FE : public TransformHomogeneous<SCALAR, Type_fr_Humerus_R_X_fr_Wrist_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Humerus_R_X_fr_Wrist_FE();
        const Type_fr_Humerus_R_X_fr_Wrist_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Elbow_FE_X_fr_Wrist_FE : public TransformHomogeneous<SCALAR, Type_fr_Elbow_FE_X_fr_Wrist_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Elbow_FE_X_fr_Wrist_FE();
        const Type_fr_Elbow_FE_X_fr_Wrist_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_R_X_fr_Wrist_FE : public TransformHomogeneous<SCALAR, Type_fr_Wrist_R_X_fr_Wrist_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_R_X_fr_Wrist_FE();
        const Type_fr_Wrist_R_X_fr_Wrist_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_FE_X_fr_Shoulder_FE : public TransformHomogeneous<SCALAR, Type_fr_Wrist_FE_X_fr_Shoulder_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_FE_X_fr_Shoulder_FE();
        const Type_fr_Wrist_FE_X_fr_Shoulder_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_FE_X_fr_ee : public TransformHomogeneous<SCALAR, Type_fr_Shoulder_FE_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_FE_X_fr_ee();
        const Type_fr_Shoulder_FE_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_Humerus_R_X_fr_ee : public TransformHomogeneous<SCALAR, Type_fr_Humerus_R_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Humerus_R_X_fr_ee();
        const Type_fr_Humerus_R_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_Elbow_FE_X_fr_ee : public TransformHomogeneous<SCALAR, Type_fr_Elbow_FE_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Elbow_FE_X_fr_ee();
        const Type_fr_Elbow_FE_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_R_X_fr_ee : public TransformHomogeneous<SCALAR, Type_fr_Wrist_R_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_R_X_fr_ee();
        const Type_fr_Wrist_R_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_FE_X_fr_ee : public TransformHomogeneous<SCALAR, Type_fr_Wrist_FE_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_FE_X_fr_ee();
        const Type_fr_Wrist_FE_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_AA_X_fr_HyABase : public TransformHomogeneous<SCALAR, Type_fr_Shoulder_AA_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_AA_X_fr_HyABase();
        const Type_fr_Shoulder_AA_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_FE_X_fr_HyABase : public TransformHomogeneous<SCALAR, Type_fr_Shoulder_FE_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_FE_X_fr_HyABase();
        const Type_fr_Shoulder_FE_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_Humerus_R_X_fr_HyABase : public TransformHomogeneous<SCALAR, Type_fr_Humerus_R_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Humerus_R_X_fr_HyABase();
        const Type_fr_Humerus_R_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_Elbow_FE_X_fr_HyABase : public TransformHomogeneous<SCALAR, Type_fr_Elbow_FE_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Elbow_FE_X_fr_HyABase();
        const Type_fr_Elbow_FE_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_R_X_fr_HyABase : public TransformHomogeneous<SCALAR, Type_fr_Wrist_R_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_R_X_fr_HyABase();
        const Type_fr_Wrist_R_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_FE_X_fr_HyABase : public TransformHomogeneous<SCALAR, Type_fr_Wrist_FE_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_FE_X_fr_HyABase();
        const Type_fr_Wrist_FE_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_ee_X_fr_HyABase : public TransformHomogeneous<SCALAR, Type_fr_ee_X_fr_HyABase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ee_X_fr_HyABase();
        const Type_fr_ee_X_fr_HyABase& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_SAA : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_SAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_SAA();
        const Type_fr_HyABase_X_fr_SAA& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_SFE : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_SFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_SFE();
        const Type_fr_HyABase_X_fr_SFE& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_HR : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_HR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_HR();
        const Type_fr_HyABase_X_fr_HR& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_EFE : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_EFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_EFE();
        const Type_fr_HyABase_X_fr_EFE& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_WR : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_WR>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_WR();
        const Type_fr_HyABase_X_fr_WR& update(const JState&);
    protected:
    };
    
    class Type_fr_HyABase_X_fr_WFE : public TransformHomogeneous<SCALAR, Type_fr_HyABase_X_fr_WFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_HyABase_X_fr_WFE();
        const Type_fr_HyABase_X_fr_WFE& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_FE_X_fr_Shoulder_AA : public TransformHomogeneous<SCALAR, Type_fr_Shoulder_FE_X_fr_Shoulder_AA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_FE_X_fr_Shoulder_AA();
        const Type_fr_Shoulder_FE_X_fr_Shoulder_AA& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_AA_X_fr_Shoulder_FE : public TransformHomogeneous<SCALAR, Type_fr_Shoulder_AA_X_fr_Shoulder_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_AA_X_fr_Shoulder_FE();
        const Type_fr_Shoulder_AA_X_fr_Shoulder_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Humerus_R_X_fr_Shoulder_FE : public TransformHomogeneous<SCALAR, Type_fr_Humerus_R_X_fr_Shoulder_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Humerus_R_X_fr_Shoulder_FE();
        const Type_fr_Humerus_R_X_fr_Shoulder_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Shoulder_FE_X_fr_Humerus_R : public TransformHomogeneous<SCALAR, Type_fr_Shoulder_FE_X_fr_Humerus_R>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Shoulder_FE_X_fr_Humerus_R();
        const Type_fr_Shoulder_FE_X_fr_Humerus_R& update(const JState&);
    protected:
    };
    
    class Type_fr_Elbow_FE_X_fr_Humerus_R : public TransformHomogeneous<SCALAR, Type_fr_Elbow_FE_X_fr_Humerus_R>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Elbow_FE_X_fr_Humerus_R();
        const Type_fr_Elbow_FE_X_fr_Humerus_R& update(const JState&);
    protected:
    };
    
    class Type_fr_Humerus_R_X_fr_Elbow_FE : public TransformHomogeneous<SCALAR, Type_fr_Humerus_R_X_fr_Elbow_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Humerus_R_X_fr_Elbow_FE();
        const Type_fr_Humerus_R_X_fr_Elbow_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_R_X_fr_Elbow_FE : public TransformHomogeneous<SCALAR, Type_fr_Wrist_R_X_fr_Elbow_FE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_R_X_fr_Elbow_FE();
        const Type_fr_Wrist_R_X_fr_Elbow_FE& update(const JState&);
    protected:
    };
    
    class Type_fr_Elbow_FE_X_fr_Wrist_R : public TransformHomogeneous<SCALAR, Type_fr_Elbow_FE_X_fr_Wrist_R>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Elbow_FE_X_fr_Wrist_R();
        const Type_fr_Elbow_FE_X_fr_Wrist_R& update(const JState&);
    protected:
    };
    
    class Type_fr_Wrist_FE_X_fr_Wrist_R : public TransformHomogeneous<SCALAR, Type_fr_Wrist_FE_X_fr_Wrist_R>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Wrist_FE_X_fr_Wrist_R();
        const Type_fr_Wrist_FE_X_fr_Wrist_R& update(const JState&);
    protected:
    };
    
public:
    HomogeneousTransforms();
    void updateParameters();
    Type_fr_HyABase_X_fr_Shoulder_AA fr_HyABase_X_fr_Shoulder_AA;
    Type_fr_HyABase_X_fr_Shoulder_FE fr_HyABase_X_fr_Shoulder_FE;
    Type_fr_HyABase_X_fr_Humerus_R fr_HyABase_X_fr_Humerus_R;
    Type_fr_HyABase_X_fr_Elbow_FE fr_HyABase_X_fr_Elbow_FE;
    Type_fr_HyABase_X_fr_Wrist_R fr_HyABase_X_fr_Wrist_R;
    Type_fr_HyABase_X_fr_Wrist_FE fr_HyABase_X_fr_Wrist_FE;
    Type_fr_HyABase_X_fr_ee fr_HyABase_X_fr_ee;
    Type_fr_HyABase_X_fr_Shoulder_AA_COM fr_HyABase_X_fr_Shoulder_AA_COM;
    Type_fr_HyABase_X_fr_Shoulder_FE_COM fr_HyABase_X_fr_Shoulder_FE_COM;
    Type_fr_HyABase_X_fr_Humerus_R_COM fr_HyABase_X_fr_Humerus_R_COM;
    Type_fr_HyABase_X_fr_Elbow_FE_COM fr_HyABase_X_fr_Elbow_FE_COM;
    Type_fr_HyABase_X_fr_Wrist_R_COM fr_HyABase_X_fr_Wrist_R_COM;
    Type_fr_HyABase_X_fr_Wrist_FE_COM fr_HyABase_X_fr_Wrist_FE_COM;
    Type_fr_HyABase_X_fr_Shoulder_AA_CTR fr_HyABase_X_fr_Shoulder_AA_CTR;
    Type_fr_HyABase_X_fr_Shoulder_FE_CTR fr_HyABase_X_fr_Shoulder_FE_CTR;
    Type_fr_HyABase_X_fr_Humerus_R_CTR fr_HyABase_X_fr_Humerus_R_CTR;
    Type_fr_HyABase_X_fr_Elbow_FE_CTR fr_HyABase_X_fr_Elbow_FE_CTR;
    Type_fr_HyABase_X_fr_Wrist_R_CTR fr_HyABase_X_fr_Wrist_R_CTR;
    Type_fr_Shoulder_FE_X_fr_Wrist_FE fr_Shoulder_FE_X_fr_Wrist_FE;
    Type_fr_Humerus_R_X_fr_Wrist_FE fr_Humerus_R_X_fr_Wrist_FE;
    Type_fr_Elbow_FE_X_fr_Wrist_FE fr_Elbow_FE_X_fr_Wrist_FE;
    Type_fr_Wrist_R_X_fr_Wrist_FE fr_Wrist_R_X_fr_Wrist_FE;
    Type_fr_Wrist_FE_X_fr_Shoulder_FE fr_Wrist_FE_X_fr_Shoulder_FE;
    Type_fr_Shoulder_FE_X_fr_ee fr_Shoulder_FE_X_fr_ee;
    Type_fr_Humerus_R_X_fr_ee fr_Humerus_R_X_fr_ee;
    Type_fr_Elbow_FE_X_fr_ee fr_Elbow_FE_X_fr_ee;
    Type_fr_Wrist_R_X_fr_ee fr_Wrist_R_X_fr_ee;
    Type_fr_Wrist_FE_X_fr_ee fr_Wrist_FE_X_fr_ee;
    Type_fr_Shoulder_AA_X_fr_HyABase fr_Shoulder_AA_X_fr_HyABase;
    Type_fr_Shoulder_FE_X_fr_HyABase fr_Shoulder_FE_X_fr_HyABase;
    Type_fr_Humerus_R_X_fr_HyABase fr_Humerus_R_X_fr_HyABase;
    Type_fr_Elbow_FE_X_fr_HyABase fr_Elbow_FE_X_fr_HyABase;
    Type_fr_Wrist_R_X_fr_HyABase fr_Wrist_R_X_fr_HyABase;
    Type_fr_Wrist_FE_X_fr_HyABase fr_Wrist_FE_X_fr_HyABase;
    Type_fr_ee_X_fr_HyABase fr_ee_X_fr_HyABase;
    Type_fr_HyABase_X_fr_SAA fr_HyABase_X_fr_SAA;
    Type_fr_HyABase_X_fr_SFE fr_HyABase_X_fr_SFE;
    Type_fr_HyABase_X_fr_HR fr_HyABase_X_fr_HR;
    Type_fr_HyABase_X_fr_EFE fr_HyABase_X_fr_EFE;
    Type_fr_HyABase_X_fr_WR fr_HyABase_X_fr_WR;
    Type_fr_HyABase_X_fr_WFE fr_HyABase_X_fr_WFE;
    Type_fr_Shoulder_FE_X_fr_Shoulder_AA fr_Shoulder_FE_X_fr_Shoulder_AA;
    Type_fr_Shoulder_AA_X_fr_Shoulder_FE fr_Shoulder_AA_X_fr_Shoulder_FE;
    Type_fr_Humerus_R_X_fr_Shoulder_FE fr_Humerus_R_X_fr_Shoulder_FE;
    Type_fr_Shoulder_FE_X_fr_Humerus_R fr_Shoulder_FE_X_fr_Humerus_R;
    Type_fr_Elbow_FE_X_fr_Humerus_R fr_Elbow_FE_X_fr_Humerus_R;
    Type_fr_Humerus_R_X_fr_Elbow_FE fr_Humerus_R_X_fr_Elbow_FE;
    Type_fr_Wrist_R_X_fr_Elbow_FE fr_Wrist_R_X_fr_Elbow_FE;
    Type_fr_Elbow_FE_X_fr_Wrist_R fr_Elbow_FE_X_fr_Wrist_R;
    Type_fr_Wrist_FE_X_fr_Wrist_R fr_Wrist_FE_X_fr_Wrist_R;

protected:

}; //class 'HomogeneousTransforms'

} // namespace tpl

using MotionTransforms = tpl::MotionTransforms<rbd::DoubleTrait>;
using ForceTransforms = tpl::ForceTransforms<rbd::DoubleTrait>;
using HomogeneousTransforms = tpl::HomogeneousTransforms<rbd::DoubleTrait>;

}
}

#include "transforms.impl.h"

#endif
