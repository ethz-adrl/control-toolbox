#ifndef CT_DOUBLEINVERTEDPENDULUM_TRANSFORMS_H_
#define CT_DOUBLEINVERTEDPENDULUM_TRANSFORMS_H_

#include <Eigen/Dense>
#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include <iit/rbd/traits/DoubleTrait.h>
#include "kinematics_parameters.h"

namespace iit {
namespace ct_DoubleInvertedPendulum {

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
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformMotion<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_DoubleInvertedPendulumBase_X_fr_Link1 : public TransformMotion<Scalar, Type_fr_DoubleInvertedPendulumBase_X_fr_Link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_DoubleInvertedPendulumBase_X_fr_Link1();
        const Type_fr_DoubleInvertedPendulumBase_X_fr_Link1& update(const JState&);
    protected:
    };
    
    class Type_fr_DoubleInvertedPendulumBase_X_fr_Link2 : public TransformMotion<Scalar, Type_fr_DoubleInvertedPendulumBase_X_fr_Link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_DoubleInvertedPendulumBase_X_fr_Link2();
        const Type_fr_DoubleInvertedPendulumBase_X_fr_Link2& update(const JState&);
    protected:
    };
    
    class Type_fr_DoubleInvertedPendulumBase_X_fr_ee : public TransformMotion<Scalar, Type_fr_DoubleInvertedPendulumBase_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_DoubleInvertedPendulumBase_X_fr_ee();
        const Type_fr_DoubleInvertedPendulumBase_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_Link2_X_fr_DoubleInvertedPendulumBase : public TransformMotion<Scalar, Type_fr_Link2_X_fr_DoubleInvertedPendulumBase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Link2_X_fr_DoubleInvertedPendulumBase();
        const Type_fr_Link2_X_fr_DoubleInvertedPendulumBase& update(const JState&);
    protected:
    };
    
    class Type_fr_Link1_X_fr_DoubleInvertedPendulumBase : public TransformMotion<Scalar, Type_fr_Link1_X_fr_DoubleInvertedPendulumBase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Link1_X_fr_DoubleInvertedPendulumBase();
        const Type_fr_Link1_X_fr_DoubleInvertedPendulumBase& update(const JState&);
    protected:
    };
    
    class Type_fr_ee_X_fr_DoubleInvertedPendulumBase : public TransformMotion<Scalar, Type_fr_ee_X_fr_DoubleInvertedPendulumBase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ee_X_fr_DoubleInvertedPendulumBase();
        const Type_fr_ee_X_fr_DoubleInvertedPendulumBase& update(const JState&);
    protected:
    };
    
    class Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1 : public TransformMotion<Scalar, Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1();
        const Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1& update(const JState&);
    protected:
    };
    
    class Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2 : public TransformMotion<Scalar, Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2();
        const Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2& update(const JState&);
    protected:
    };
    
    class Type_fr_Link2_X_fr_Link1 : public TransformMotion<Scalar, Type_fr_Link2_X_fr_Link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Link2_X_fr_Link1();
        const Type_fr_Link2_X_fr_Link1& update(const JState&);
    protected:
    };
    
    class Type_fr_Link1_X_fr_Link2 : public TransformMotion<Scalar, Type_fr_Link1_X_fr_Link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Link1_X_fr_Link2();
        const Type_fr_Link1_X_fr_Link2& update(const JState&);
    protected:
    };
    
public:
    MotionTransforms();
    void updateParameters();
    Type_fr_DoubleInvertedPendulumBase_X_fr_Link1 fr_DoubleInvertedPendulumBase_X_fr_Link1;
    Type_fr_DoubleInvertedPendulumBase_X_fr_Link2 fr_DoubleInvertedPendulumBase_X_fr_Link2;
    Type_fr_DoubleInvertedPendulumBase_X_fr_ee fr_DoubleInvertedPendulumBase_X_fr_ee;
    Type_fr_Link2_X_fr_DoubleInvertedPendulumBase fr_Link2_X_fr_DoubleInvertedPendulumBase;
    Type_fr_Link1_X_fr_DoubleInvertedPendulumBase fr_Link1_X_fr_DoubleInvertedPendulumBase;
    Type_fr_ee_X_fr_DoubleInvertedPendulumBase fr_ee_X_fr_DoubleInvertedPendulumBase;
    Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1 fr_DoubleInvertedPendulumBase_X_fr_Joint1;
    Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2 fr_DoubleInvertedPendulumBase_X_fr_Joint2;
    Type_fr_Link2_X_fr_Link1 fr_Link2_X_fr_Link1;
    Type_fr_Link1_X_fr_Link2 fr_Link1_X_fr_Link2;

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
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformForce<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_DoubleInvertedPendulumBase_X_fr_Link1 : public TransformForce<Scalar, Type_fr_DoubleInvertedPendulumBase_X_fr_Link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_DoubleInvertedPendulumBase_X_fr_Link1();
        const Type_fr_DoubleInvertedPendulumBase_X_fr_Link1& update(const JState&);
    protected:
    };
    
    class Type_fr_DoubleInvertedPendulumBase_X_fr_Link2 : public TransformForce<Scalar, Type_fr_DoubleInvertedPendulumBase_X_fr_Link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_DoubleInvertedPendulumBase_X_fr_Link2();
        const Type_fr_DoubleInvertedPendulumBase_X_fr_Link2& update(const JState&);
    protected:
    };
    
    class Type_fr_DoubleInvertedPendulumBase_X_fr_ee : public TransformForce<Scalar, Type_fr_DoubleInvertedPendulumBase_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_DoubleInvertedPendulumBase_X_fr_ee();
        const Type_fr_DoubleInvertedPendulumBase_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_Link2_X_fr_DoubleInvertedPendulumBase : public TransformForce<Scalar, Type_fr_Link2_X_fr_DoubleInvertedPendulumBase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Link2_X_fr_DoubleInvertedPendulumBase();
        const Type_fr_Link2_X_fr_DoubleInvertedPendulumBase& update(const JState&);
    protected:
    };
    
    class Type_fr_Link1_X_fr_DoubleInvertedPendulumBase : public TransformForce<Scalar, Type_fr_Link1_X_fr_DoubleInvertedPendulumBase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Link1_X_fr_DoubleInvertedPendulumBase();
        const Type_fr_Link1_X_fr_DoubleInvertedPendulumBase& update(const JState&);
    protected:
    };
    
    class Type_fr_ee_X_fr_DoubleInvertedPendulumBase : public TransformForce<Scalar, Type_fr_ee_X_fr_DoubleInvertedPendulumBase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ee_X_fr_DoubleInvertedPendulumBase();
        const Type_fr_ee_X_fr_DoubleInvertedPendulumBase& update(const JState&);
    protected:
    };
    
    class Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1 : public TransformForce<Scalar, Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1();
        const Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1& update(const JState&);
    protected:
    };
    
    class Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2 : public TransformForce<Scalar, Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2();
        const Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2& update(const JState&);
    protected:
    };
    
    class Type_fr_Link2_X_fr_Link1 : public TransformForce<Scalar, Type_fr_Link2_X_fr_Link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Link2_X_fr_Link1();
        const Type_fr_Link2_X_fr_Link1& update(const JState&);
    protected:
    };
    
    class Type_fr_Link1_X_fr_Link2 : public TransformForce<Scalar, Type_fr_Link1_X_fr_Link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Link1_X_fr_Link2();
        const Type_fr_Link1_X_fr_Link2& update(const JState&);
    protected:
    };
    
public:
    ForceTransforms();
    void updateParameters();
    Type_fr_DoubleInvertedPendulumBase_X_fr_Link1 fr_DoubleInvertedPendulumBase_X_fr_Link1;
    Type_fr_DoubleInvertedPendulumBase_X_fr_Link2 fr_DoubleInvertedPendulumBase_X_fr_Link2;
    Type_fr_DoubleInvertedPendulumBase_X_fr_ee fr_DoubleInvertedPendulumBase_X_fr_ee;
    Type_fr_Link2_X_fr_DoubleInvertedPendulumBase fr_Link2_X_fr_DoubleInvertedPendulumBase;
    Type_fr_Link1_X_fr_DoubleInvertedPendulumBase fr_Link1_X_fr_DoubleInvertedPendulumBase;
    Type_fr_ee_X_fr_DoubleInvertedPendulumBase fr_ee_X_fr_DoubleInvertedPendulumBase;
    Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1 fr_DoubleInvertedPendulumBase_X_fr_Joint1;
    Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2 fr_DoubleInvertedPendulumBase_X_fr_Joint2;
    Type_fr_Link2_X_fr_Link1 fr_Link2_X_fr_Link1;
    Type_fr_Link1_X_fr_Link2 fr_Link1_X_fr_Link2;

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
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformHomogeneous<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_DoubleInvertedPendulumBase_X_fr_Link1 : public TransformHomogeneous<Scalar, Type_fr_DoubleInvertedPendulumBase_X_fr_Link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_DoubleInvertedPendulumBase_X_fr_Link1();
        const Type_fr_DoubleInvertedPendulumBase_X_fr_Link1& update(const JState&);
    protected:
    };
    
    class Type_fr_DoubleInvertedPendulumBase_X_fr_Link2 : public TransformHomogeneous<Scalar, Type_fr_DoubleInvertedPendulumBase_X_fr_Link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_DoubleInvertedPendulumBase_X_fr_Link2();
        const Type_fr_DoubleInvertedPendulumBase_X_fr_Link2& update(const JState&);
    protected:
    };
    
    class Type_fr_DoubleInvertedPendulumBase_X_fr_ee : public TransformHomogeneous<Scalar, Type_fr_DoubleInvertedPendulumBase_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_DoubleInvertedPendulumBase_X_fr_ee();
        const Type_fr_DoubleInvertedPendulumBase_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_Link2_X_fr_DoubleInvertedPendulumBase : public TransformHomogeneous<Scalar, Type_fr_Link2_X_fr_DoubleInvertedPendulumBase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Link2_X_fr_DoubleInvertedPendulumBase();
        const Type_fr_Link2_X_fr_DoubleInvertedPendulumBase& update(const JState&);
    protected:
    };
    
    class Type_fr_Link1_X_fr_DoubleInvertedPendulumBase : public TransformHomogeneous<Scalar, Type_fr_Link1_X_fr_DoubleInvertedPendulumBase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Link1_X_fr_DoubleInvertedPendulumBase();
        const Type_fr_Link1_X_fr_DoubleInvertedPendulumBase& update(const JState&);
    protected:
    };
    
    class Type_fr_ee_X_fr_DoubleInvertedPendulumBase : public TransformHomogeneous<Scalar, Type_fr_ee_X_fr_DoubleInvertedPendulumBase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ee_X_fr_DoubleInvertedPendulumBase();
        const Type_fr_ee_X_fr_DoubleInvertedPendulumBase& update(const JState&);
    protected:
    };
    
    class Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1 : public TransformHomogeneous<Scalar, Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1();
        const Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1& update(const JState&);
    protected:
    };
    
    class Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2 : public TransformHomogeneous<Scalar, Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2();
        const Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2& update(const JState&);
    protected:
    };
    
    class Type_fr_Link2_X_fr_Link1 : public TransformHomogeneous<Scalar, Type_fr_Link2_X_fr_Link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Link2_X_fr_Link1();
        const Type_fr_Link2_X_fr_Link1& update(const JState&);
    protected:
    };
    
    class Type_fr_Link1_X_fr_Link2 : public TransformHomogeneous<Scalar, Type_fr_Link1_X_fr_Link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_Link1_X_fr_Link2();
        const Type_fr_Link1_X_fr_Link2& update(const JState&);
    protected:
    };
    
public:
    HomogeneousTransforms();
    void updateParameters();
    Type_fr_DoubleInvertedPendulumBase_X_fr_Link1 fr_DoubleInvertedPendulumBase_X_fr_Link1;
    Type_fr_DoubleInvertedPendulumBase_X_fr_Link2 fr_DoubleInvertedPendulumBase_X_fr_Link2;
    Type_fr_DoubleInvertedPendulumBase_X_fr_ee fr_DoubleInvertedPendulumBase_X_fr_ee;
    Type_fr_Link2_X_fr_DoubleInvertedPendulumBase fr_Link2_X_fr_DoubleInvertedPendulumBase;
    Type_fr_Link1_X_fr_DoubleInvertedPendulumBase fr_Link1_X_fr_DoubleInvertedPendulumBase;
    Type_fr_ee_X_fr_DoubleInvertedPendulumBase fr_ee_X_fr_DoubleInvertedPendulumBase;
    Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1 fr_DoubleInvertedPendulumBase_X_fr_Joint1;
    Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2 fr_DoubleInvertedPendulumBase_X_fr_Joint2;
    Type_fr_Link2_X_fr_Link1 fr_Link2_X_fr_Link1;
    Type_fr_Link1_X_fr_Link2 fr_Link1_X_fr_Link2;

protected:

}; //class 'HomogeneousTransforms'

}

using MotionTransforms = tpl::MotionTransforms<rbd::DoubleTrait>;
using ForceTransforms = tpl::ForceTransforms<rbd::DoubleTrait>;
using HomogeneousTransforms = tpl::HomogeneousTransforms<rbd::DoubleTrait>;

}
}

#include "transforms.impl.h"

#endif
