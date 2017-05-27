#ifndef CT_QUADROTOR_TRANSFORMS_H_
#define CT_QUADROTOR_TRANSFORMS_H_

#include <Eigen/Dense>
#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include <iit/rbd/traits/DoubleTrait.h>
#include "kinematics_parameters.h"

namespace iit {
namespace ct_quadrotor {

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
    class Type_fr_body_X_fr_link1 : public TransformMotion<SCALAR, Type_fr_body_X_fr_link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_link1();
        const Type_fr_body_X_fr_link1& update(const JState&);
    protected:
    };
    
    class Type_fr_body_X_fr_link2 : public TransformMotion<SCALAR, Type_fr_body_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_link2();
        const Type_fr_body_X_fr_link2& update(const JState&);
    protected:
    };
    
    class Type_fr_body_X_fr_ee : public TransformMotion<SCALAR, Type_fr_body_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_ee();
        const Type_fr_body_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_body_X_fr_com0 : public TransformMotion<SCALAR, Type_fr_body_X_fr_com0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_com0();
        const Type_fr_body_X_fr_com0& update(const JState&);
    protected:
    };
    
    class Type_fr_body_X_fr_com1 : public TransformMotion<SCALAR, Type_fr_body_X_fr_com1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_com1();
        const Type_fr_body_X_fr_com1& update(const JState&);
    protected:
    };
    
    class Type_fr_link1_X_fr_body : public TransformMotion<SCALAR, Type_fr_link1_X_fr_body>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link1_X_fr_body();
        const Type_fr_link1_X_fr_body& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_body : public TransformMotion<SCALAR, Type_fr_link2_X_fr_body>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_body();
        const Type_fr_link2_X_fr_body& update(const JState&);
    protected:
    };
    
    class Type_fr_ee_X_fr_body : public TransformMotion<SCALAR, Type_fr_ee_X_fr_body>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ee_X_fr_body();
        const Type_fr_ee_X_fr_body& update(const JState&);
    protected:
    };
    
    class Type_fr_com0_X_fr_body : public TransformMotion<SCALAR, Type_fr_com0_X_fr_body>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_com0_X_fr_body();
        const Type_fr_com0_X_fr_body& update(const JState&);
    protected:
    };
    
    class Type_fr_com1_X_fr_body : public TransformMotion<SCALAR, Type_fr_com1_X_fr_body>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_com1_X_fr_body();
        const Type_fr_com1_X_fr_body& update(const JState&);
    protected:
    };
    
    class Type_fr_body_X_fr_jA : public TransformMotion<SCALAR, Type_fr_body_X_fr_jA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_jA();
        const Type_fr_body_X_fr_jA& update(const JState&);
    protected:
    };
    
    class Type_fr_body_X_fr_jB : public TransformMotion<SCALAR, Type_fr_body_X_fr_jB>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_jB();
        const Type_fr_body_X_fr_jB& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_link1 : public TransformMotion<SCALAR, Type_fr_link2_X_fr_link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_link1();
        const Type_fr_link2_X_fr_link1& update(const JState&);
    protected:
    };
    
    class Type_fr_link1_X_fr_link2 : public TransformMotion<SCALAR, Type_fr_link1_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link1_X_fr_link2();
        const Type_fr_link1_X_fr_link2& update(const JState&);
    protected:
    };
    
public:
    MotionTransforms();
    void updateParameters();
    Type_fr_body_X_fr_link1 fr_body_X_fr_link1;
    Type_fr_body_X_fr_link2 fr_body_X_fr_link2;
    Type_fr_body_X_fr_ee fr_body_X_fr_ee;
    Type_fr_body_X_fr_com0 fr_body_X_fr_com0;
    Type_fr_body_X_fr_com1 fr_body_X_fr_com1;
    Type_fr_link1_X_fr_body fr_link1_X_fr_body;
    Type_fr_link2_X_fr_body fr_link2_X_fr_body;
    Type_fr_ee_X_fr_body fr_ee_X_fr_body;
    Type_fr_com0_X_fr_body fr_com0_X_fr_body;
    Type_fr_com1_X_fr_body fr_com1_X_fr_body;
    Type_fr_body_X_fr_jA fr_body_X_fr_jA;
    Type_fr_body_X_fr_jB fr_body_X_fr_jB;
    Type_fr_link2_X_fr_link1 fr_link2_X_fr_link1;
    Type_fr_link1_X_fr_link2 fr_link1_X_fr_link2;

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
    class Type_fr_body_X_fr_link1 : public TransformForce<SCALAR, Type_fr_body_X_fr_link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_link1();
        const Type_fr_body_X_fr_link1& update(const JState&);
    protected:
    };
    
    class Type_fr_body_X_fr_link2 : public TransformForce<SCALAR, Type_fr_body_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_link2();
        const Type_fr_body_X_fr_link2& update(const JState&);
    protected:
    };
    
    class Type_fr_body_X_fr_ee : public TransformForce<SCALAR, Type_fr_body_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_ee();
        const Type_fr_body_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_body_X_fr_com0 : public TransformForce<SCALAR, Type_fr_body_X_fr_com0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_com0();
        const Type_fr_body_X_fr_com0& update(const JState&);
    protected:
    };
    
    class Type_fr_body_X_fr_com1 : public TransformForce<SCALAR, Type_fr_body_X_fr_com1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_com1();
        const Type_fr_body_X_fr_com1& update(const JState&);
    protected:
    };
    
    class Type_fr_link1_X_fr_body : public TransformForce<SCALAR, Type_fr_link1_X_fr_body>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link1_X_fr_body();
        const Type_fr_link1_X_fr_body& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_body : public TransformForce<SCALAR, Type_fr_link2_X_fr_body>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_body();
        const Type_fr_link2_X_fr_body& update(const JState&);
    protected:
    };
    
    class Type_fr_ee_X_fr_body : public TransformForce<SCALAR, Type_fr_ee_X_fr_body>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ee_X_fr_body();
        const Type_fr_ee_X_fr_body& update(const JState&);
    protected:
    };
    
    class Type_fr_com0_X_fr_body : public TransformForce<SCALAR, Type_fr_com0_X_fr_body>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_com0_X_fr_body();
        const Type_fr_com0_X_fr_body& update(const JState&);
    protected:
    };
    
    class Type_fr_com1_X_fr_body : public TransformForce<SCALAR, Type_fr_com1_X_fr_body>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_com1_X_fr_body();
        const Type_fr_com1_X_fr_body& update(const JState&);
    protected:
    };
    
    class Type_fr_body_X_fr_jA : public TransformForce<SCALAR, Type_fr_body_X_fr_jA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_jA();
        const Type_fr_body_X_fr_jA& update(const JState&);
    protected:
    };
    
    class Type_fr_body_X_fr_jB : public TransformForce<SCALAR, Type_fr_body_X_fr_jB>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_jB();
        const Type_fr_body_X_fr_jB& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_link1 : public TransformForce<SCALAR, Type_fr_link2_X_fr_link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_link1();
        const Type_fr_link2_X_fr_link1& update(const JState&);
    protected:
    };
    
    class Type_fr_link1_X_fr_link2 : public TransformForce<SCALAR, Type_fr_link1_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link1_X_fr_link2();
        const Type_fr_link1_X_fr_link2& update(const JState&);
    protected:
    };
    
public:
    ForceTransforms();
    void updateParameters();
    Type_fr_body_X_fr_link1 fr_body_X_fr_link1;
    Type_fr_body_X_fr_link2 fr_body_X_fr_link2;
    Type_fr_body_X_fr_ee fr_body_X_fr_ee;
    Type_fr_body_X_fr_com0 fr_body_X_fr_com0;
    Type_fr_body_X_fr_com1 fr_body_X_fr_com1;
    Type_fr_link1_X_fr_body fr_link1_X_fr_body;
    Type_fr_link2_X_fr_body fr_link2_X_fr_body;
    Type_fr_ee_X_fr_body fr_ee_X_fr_body;
    Type_fr_com0_X_fr_body fr_com0_X_fr_body;
    Type_fr_com1_X_fr_body fr_com1_X_fr_body;
    Type_fr_body_X_fr_jA fr_body_X_fr_jA;
    Type_fr_body_X_fr_jB fr_body_X_fr_jB;
    Type_fr_link2_X_fr_link1 fr_link2_X_fr_link1;
    Type_fr_link1_X_fr_link2 fr_link1_X_fr_link2;

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
    class Type_fr_body_X_fr_link1 : public TransformHomogeneous<SCALAR, Type_fr_body_X_fr_link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_link1();
        const Type_fr_body_X_fr_link1& update(const JState&);
    protected:
    };
    
    class Type_fr_body_X_fr_link2 : public TransformHomogeneous<SCALAR, Type_fr_body_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_link2();
        const Type_fr_body_X_fr_link2& update(const JState&);
    protected:
    };
    
    class Type_fr_body_X_fr_ee : public TransformHomogeneous<SCALAR, Type_fr_body_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_ee();
        const Type_fr_body_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_body_X_fr_com0 : public TransformHomogeneous<SCALAR, Type_fr_body_X_fr_com0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_com0();
        const Type_fr_body_X_fr_com0& update(const JState&);
    protected:
    };
    
    class Type_fr_body_X_fr_com1 : public TransformHomogeneous<SCALAR, Type_fr_body_X_fr_com1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_com1();
        const Type_fr_body_X_fr_com1& update(const JState&);
    protected:
    };
    
    class Type_fr_link1_X_fr_body : public TransformHomogeneous<SCALAR, Type_fr_link1_X_fr_body>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link1_X_fr_body();
        const Type_fr_link1_X_fr_body& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_body : public TransformHomogeneous<SCALAR, Type_fr_link2_X_fr_body>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_body();
        const Type_fr_link2_X_fr_body& update(const JState&);
    protected:
    };
    
    class Type_fr_ee_X_fr_body : public TransformHomogeneous<SCALAR, Type_fr_ee_X_fr_body>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ee_X_fr_body();
        const Type_fr_ee_X_fr_body& update(const JState&);
    protected:
    };
    
    class Type_fr_com0_X_fr_body : public TransformHomogeneous<SCALAR, Type_fr_com0_X_fr_body>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_com0_X_fr_body();
        const Type_fr_com0_X_fr_body& update(const JState&);
    protected:
    };
    
    class Type_fr_com1_X_fr_body : public TransformHomogeneous<SCALAR, Type_fr_com1_X_fr_body>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_com1_X_fr_body();
        const Type_fr_com1_X_fr_body& update(const JState&);
    protected:
    };
    
    class Type_fr_body_X_fr_jA : public TransformHomogeneous<SCALAR, Type_fr_body_X_fr_jA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_jA();
        const Type_fr_body_X_fr_jA& update(const JState&);
    protected:
    };
    
    class Type_fr_body_X_fr_jB : public TransformHomogeneous<SCALAR, Type_fr_body_X_fr_jB>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_body_X_fr_jB();
        const Type_fr_body_X_fr_jB& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_link1 : public TransformHomogeneous<SCALAR, Type_fr_link2_X_fr_link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_link1();
        const Type_fr_link2_X_fr_link1& update(const JState&);
    protected:
    };
    
    class Type_fr_link1_X_fr_link2 : public TransformHomogeneous<SCALAR, Type_fr_link1_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link1_X_fr_link2();
        const Type_fr_link1_X_fr_link2& update(const JState&);
    protected:
    };
    
public:
    HomogeneousTransforms();
    void updateParameters();
    Type_fr_body_X_fr_link1 fr_body_X_fr_link1;
    Type_fr_body_X_fr_link2 fr_body_X_fr_link2;
    Type_fr_body_X_fr_ee fr_body_X_fr_ee;
    Type_fr_body_X_fr_com0 fr_body_X_fr_com0;
    Type_fr_body_X_fr_com1 fr_body_X_fr_com1;
    Type_fr_link1_X_fr_body fr_link1_X_fr_body;
    Type_fr_link2_X_fr_body fr_link2_X_fr_body;
    Type_fr_ee_X_fr_body fr_ee_X_fr_body;
    Type_fr_com0_X_fr_body fr_com0_X_fr_body;
    Type_fr_com1_X_fr_body fr_com1_X_fr_body;
    Type_fr_body_X_fr_jA fr_body_X_fr_jA;
    Type_fr_body_X_fr_jB fr_body_X_fr_jB;
    Type_fr_link2_X_fr_link1 fr_link2_X_fr_link1;
    Type_fr_link1_X_fr_link2 fr_link1_X_fr_link2;

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
