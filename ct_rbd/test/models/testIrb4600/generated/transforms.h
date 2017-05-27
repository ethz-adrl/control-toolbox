#ifndef TESTIRB4600_TRANSFORMS_H_
#define TESTIRB4600_TRANSFORMS_H_

#include <Eigen/Dense>
#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include <iit/rbd/traits/DoubleTrait.h>
#include "kinematics_parameters.h"

namespace iit {
namespace testirb4600 {

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
    class Type_fr_link0_X_fr_link1 : public TransformMotion<SCALAR, Type_fr_link0_X_fr_link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_link1();
        const Type_fr_link0_X_fr_link1& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_link2 : public TransformMotion<SCALAR, Type_fr_link0_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_link2();
        const Type_fr_link0_X_fr_link2& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_link3 : public TransformMotion<SCALAR, Type_fr_link0_X_fr_link3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_link3();
        const Type_fr_link0_X_fr_link3& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_link4 : public TransformMotion<SCALAR, Type_fr_link0_X_fr_link4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_link4();
        const Type_fr_link0_X_fr_link4& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_link5 : public TransformMotion<SCALAR, Type_fr_link0_X_fr_link5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_link5();
        const Type_fr_link0_X_fr_link5& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_link6 : public TransformMotion<SCALAR, Type_fr_link0_X_fr_link6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_link6();
        const Type_fr_link0_X_fr_link6& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_ee : public TransformMotion<SCALAR, Type_fr_link0_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_ee();
        const Type_fr_link0_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com0 : public TransformMotion<SCALAR, Type_fr_link0_X_com0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com0();
        const Type_fr_link0_X_com0& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com1 : public TransformMotion<SCALAR, Type_fr_link0_X_com1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com1();
        const Type_fr_link0_X_com1& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com2 : public TransformMotion<SCALAR, Type_fr_link0_X_com2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com2();
        const Type_fr_link0_X_com2& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com3 : public TransformMotion<SCALAR, Type_fr_link0_X_com3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com3();
        const Type_fr_link0_X_com3& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com4 : public TransformMotion<SCALAR, Type_fr_link0_X_com4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com4();
        const Type_fr_link0_X_com4& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com5 : public TransformMotion<SCALAR, Type_fr_link0_X_com5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com5();
        const Type_fr_link0_X_com5& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com6 : public TransformMotion<SCALAR, Type_fr_link0_X_com6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com6();
        const Type_fr_link0_X_com6& update(const JState&);
    protected:
    };
    
    class Type_fr_link1_X_fr_link0 : public TransformMotion<SCALAR, Type_fr_link1_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link1_X_fr_link0();
        const Type_fr_link1_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_link0 : public TransformMotion<SCALAR, Type_fr_link2_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_link0();
        const Type_fr_link2_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_link3_X_fr_link0 : public TransformMotion<SCALAR, Type_fr_link3_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link3_X_fr_link0();
        const Type_fr_link3_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_link4_X_fr_link0 : public TransformMotion<SCALAR, Type_fr_link4_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link4_X_fr_link0();
        const Type_fr_link4_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_link5_X_fr_link0 : public TransformMotion<SCALAR, Type_fr_link5_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link5_X_fr_link0();
        const Type_fr_link5_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_link6_X_fr_link0 : public TransformMotion<SCALAR, Type_fr_link6_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link6_X_fr_link0();
        const Type_fr_link6_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_ee_X_fr_link0 : public TransformMotion<SCALAR, Type_fr_ee_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ee_X_fr_link0();
        const Type_fr_ee_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com0_X_fr_link0 : public TransformMotion<SCALAR, Type_com0_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com0_X_fr_link0();
        const Type_com0_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com1_X_fr_link0 : public TransformMotion<SCALAR, Type_com1_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com1_X_fr_link0();
        const Type_com1_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com2_X_fr_link0 : public TransformMotion<SCALAR, Type_com2_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com2_X_fr_link0();
        const Type_com2_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com3_X_fr_link0 : public TransformMotion<SCALAR, Type_com3_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com3_X_fr_link0();
        const Type_com3_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com4_X_fr_link0 : public TransformMotion<SCALAR, Type_com4_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com4_X_fr_link0();
        const Type_com4_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com5_X_fr_link0 : public TransformMotion<SCALAR, Type_com5_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com5_X_fr_link0();
        const Type_com5_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com6_X_fr_link0 : public TransformMotion<SCALAR, Type_com6_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com6_X_fr_link0();
        const Type_com6_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_jA : public TransformMotion<SCALAR, Type_fr_link0_X_fr_jA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_jA();
        const Type_fr_link0_X_fr_jA& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_jB : public TransformMotion<SCALAR, Type_fr_link0_X_fr_jB>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_jB();
        const Type_fr_link0_X_fr_jB& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_jC : public TransformMotion<SCALAR, Type_fr_link0_X_fr_jC>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_jC();
        const Type_fr_link0_X_fr_jC& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_jD : public TransformMotion<SCALAR, Type_fr_link0_X_fr_jD>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_jD();
        const Type_fr_link0_X_fr_jD& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_jE : public TransformMotion<SCALAR, Type_fr_link0_X_fr_jE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_jE();
        const Type_fr_link0_X_fr_jE& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_jF : public TransformMotion<SCALAR, Type_fr_link0_X_fr_jF>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_jF();
        const Type_fr_link0_X_fr_jF& update(const JState&);
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
    
    class Type_fr_link3_X_fr_link2 : public TransformMotion<SCALAR, Type_fr_link3_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link3_X_fr_link2();
        const Type_fr_link3_X_fr_link2& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_link3 : public TransformMotion<SCALAR, Type_fr_link2_X_fr_link3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_link3();
        const Type_fr_link2_X_fr_link3& update(const JState&);
    protected:
    };
    
    class Type_fr_link4_X_fr_link3 : public TransformMotion<SCALAR, Type_fr_link4_X_fr_link3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link4_X_fr_link3();
        const Type_fr_link4_X_fr_link3& update(const JState&);
    protected:
    };
    
    class Type_fr_link3_X_fr_link4 : public TransformMotion<SCALAR, Type_fr_link3_X_fr_link4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link3_X_fr_link4();
        const Type_fr_link3_X_fr_link4& update(const JState&);
    protected:
    };
    
    class Type_fr_link5_X_fr_link4 : public TransformMotion<SCALAR, Type_fr_link5_X_fr_link4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link5_X_fr_link4();
        const Type_fr_link5_X_fr_link4& update(const JState&);
    protected:
    };
    
    class Type_fr_link4_X_fr_link5 : public TransformMotion<SCALAR, Type_fr_link4_X_fr_link5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link4_X_fr_link5();
        const Type_fr_link4_X_fr_link5& update(const JState&);
    protected:
    };
    
    class Type_fr_link6_X_fr_link5 : public TransformMotion<SCALAR, Type_fr_link6_X_fr_link5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link6_X_fr_link5();
        const Type_fr_link6_X_fr_link5& update(const JState&);
    protected:
    };
    
    class Type_fr_link5_X_fr_link6 : public TransformMotion<SCALAR, Type_fr_link5_X_fr_link6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link5_X_fr_link6();
        const Type_fr_link5_X_fr_link6& update(const JState&);
    protected:
    };
    
public:
    MotionTransforms();
    void updateParameters();
    Type_fr_link0_X_fr_link1 fr_link0_X_fr_link1;
    Type_fr_link0_X_fr_link2 fr_link0_X_fr_link2;
    Type_fr_link0_X_fr_link3 fr_link0_X_fr_link3;
    Type_fr_link0_X_fr_link4 fr_link0_X_fr_link4;
    Type_fr_link0_X_fr_link5 fr_link0_X_fr_link5;
    Type_fr_link0_X_fr_link6 fr_link0_X_fr_link6;
    Type_fr_link0_X_fr_ee fr_link0_X_fr_ee;
    Type_fr_link0_X_com0 fr_link0_X_com0;
    Type_fr_link0_X_com1 fr_link0_X_com1;
    Type_fr_link0_X_com2 fr_link0_X_com2;
    Type_fr_link0_X_com3 fr_link0_X_com3;
    Type_fr_link0_X_com4 fr_link0_X_com4;
    Type_fr_link0_X_com5 fr_link0_X_com5;
    Type_fr_link0_X_com6 fr_link0_X_com6;
    Type_fr_link1_X_fr_link0 fr_link1_X_fr_link0;
    Type_fr_link2_X_fr_link0 fr_link2_X_fr_link0;
    Type_fr_link3_X_fr_link0 fr_link3_X_fr_link0;
    Type_fr_link4_X_fr_link0 fr_link4_X_fr_link0;
    Type_fr_link5_X_fr_link0 fr_link5_X_fr_link0;
    Type_fr_link6_X_fr_link0 fr_link6_X_fr_link0;
    Type_fr_ee_X_fr_link0 fr_ee_X_fr_link0;
    Type_com0_X_fr_link0 com0_X_fr_link0;
    Type_com1_X_fr_link0 com1_X_fr_link0;
    Type_com2_X_fr_link0 com2_X_fr_link0;
    Type_com3_X_fr_link0 com3_X_fr_link0;
    Type_com4_X_fr_link0 com4_X_fr_link0;
    Type_com5_X_fr_link0 com5_X_fr_link0;
    Type_com6_X_fr_link0 com6_X_fr_link0;
    Type_fr_link0_X_fr_jA fr_link0_X_fr_jA;
    Type_fr_link0_X_fr_jB fr_link0_X_fr_jB;
    Type_fr_link0_X_fr_jC fr_link0_X_fr_jC;
    Type_fr_link0_X_fr_jD fr_link0_X_fr_jD;
    Type_fr_link0_X_fr_jE fr_link0_X_fr_jE;
    Type_fr_link0_X_fr_jF fr_link0_X_fr_jF;
    Type_fr_link2_X_fr_link1 fr_link2_X_fr_link1;
    Type_fr_link1_X_fr_link2 fr_link1_X_fr_link2;
    Type_fr_link3_X_fr_link2 fr_link3_X_fr_link2;
    Type_fr_link2_X_fr_link3 fr_link2_X_fr_link3;
    Type_fr_link4_X_fr_link3 fr_link4_X_fr_link3;
    Type_fr_link3_X_fr_link4 fr_link3_X_fr_link4;
    Type_fr_link5_X_fr_link4 fr_link5_X_fr_link4;
    Type_fr_link4_X_fr_link5 fr_link4_X_fr_link5;
    Type_fr_link6_X_fr_link5 fr_link6_X_fr_link5;
    Type_fr_link5_X_fr_link6 fr_link5_X_fr_link6;

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
    class Type_fr_link0_X_fr_link1 : public TransformForce<SCALAR, Type_fr_link0_X_fr_link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_link1();
        const Type_fr_link0_X_fr_link1& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_link2 : public TransformForce<SCALAR, Type_fr_link0_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_link2();
        const Type_fr_link0_X_fr_link2& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_link3 : public TransformForce<SCALAR, Type_fr_link0_X_fr_link3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_link3();
        const Type_fr_link0_X_fr_link3& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_link4 : public TransformForce<SCALAR, Type_fr_link0_X_fr_link4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_link4();
        const Type_fr_link0_X_fr_link4& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_link5 : public TransformForce<SCALAR, Type_fr_link0_X_fr_link5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_link5();
        const Type_fr_link0_X_fr_link5& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_link6 : public TransformForce<SCALAR, Type_fr_link0_X_fr_link6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_link6();
        const Type_fr_link0_X_fr_link6& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_ee : public TransformForce<SCALAR, Type_fr_link0_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_ee();
        const Type_fr_link0_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com0 : public TransformForce<SCALAR, Type_fr_link0_X_com0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com0();
        const Type_fr_link0_X_com0& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com1 : public TransformForce<SCALAR, Type_fr_link0_X_com1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com1();
        const Type_fr_link0_X_com1& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com2 : public TransformForce<SCALAR, Type_fr_link0_X_com2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com2();
        const Type_fr_link0_X_com2& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com3 : public TransformForce<SCALAR, Type_fr_link0_X_com3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com3();
        const Type_fr_link0_X_com3& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com4 : public TransformForce<SCALAR, Type_fr_link0_X_com4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com4();
        const Type_fr_link0_X_com4& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com5 : public TransformForce<SCALAR, Type_fr_link0_X_com5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com5();
        const Type_fr_link0_X_com5& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com6 : public TransformForce<SCALAR, Type_fr_link0_X_com6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com6();
        const Type_fr_link0_X_com6& update(const JState&);
    protected:
    };
    
    class Type_fr_link1_X_fr_link0 : public TransformForce<SCALAR, Type_fr_link1_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link1_X_fr_link0();
        const Type_fr_link1_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_link0 : public TransformForce<SCALAR, Type_fr_link2_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_link0();
        const Type_fr_link2_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_link3_X_fr_link0 : public TransformForce<SCALAR, Type_fr_link3_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link3_X_fr_link0();
        const Type_fr_link3_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_link4_X_fr_link0 : public TransformForce<SCALAR, Type_fr_link4_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link4_X_fr_link0();
        const Type_fr_link4_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_link5_X_fr_link0 : public TransformForce<SCALAR, Type_fr_link5_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link5_X_fr_link0();
        const Type_fr_link5_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_link6_X_fr_link0 : public TransformForce<SCALAR, Type_fr_link6_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link6_X_fr_link0();
        const Type_fr_link6_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_ee_X_fr_link0 : public TransformForce<SCALAR, Type_fr_ee_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ee_X_fr_link0();
        const Type_fr_ee_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com0_X_fr_link0 : public TransformForce<SCALAR, Type_com0_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com0_X_fr_link0();
        const Type_com0_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com1_X_fr_link0 : public TransformForce<SCALAR, Type_com1_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com1_X_fr_link0();
        const Type_com1_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com2_X_fr_link0 : public TransformForce<SCALAR, Type_com2_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com2_X_fr_link0();
        const Type_com2_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com3_X_fr_link0 : public TransformForce<SCALAR, Type_com3_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com3_X_fr_link0();
        const Type_com3_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com4_X_fr_link0 : public TransformForce<SCALAR, Type_com4_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com4_X_fr_link0();
        const Type_com4_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com5_X_fr_link0 : public TransformForce<SCALAR, Type_com5_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com5_X_fr_link0();
        const Type_com5_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com6_X_fr_link0 : public TransformForce<SCALAR, Type_com6_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com6_X_fr_link0();
        const Type_com6_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_jA : public TransformForce<SCALAR, Type_fr_link0_X_fr_jA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_jA();
        const Type_fr_link0_X_fr_jA& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_jB : public TransformForce<SCALAR, Type_fr_link0_X_fr_jB>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_jB();
        const Type_fr_link0_X_fr_jB& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_jC : public TransformForce<SCALAR, Type_fr_link0_X_fr_jC>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_jC();
        const Type_fr_link0_X_fr_jC& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_jD : public TransformForce<SCALAR, Type_fr_link0_X_fr_jD>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_jD();
        const Type_fr_link0_X_fr_jD& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_jE : public TransformForce<SCALAR, Type_fr_link0_X_fr_jE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_jE();
        const Type_fr_link0_X_fr_jE& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_jF : public TransformForce<SCALAR, Type_fr_link0_X_fr_jF>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_jF();
        const Type_fr_link0_X_fr_jF& update(const JState&);
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
    
    class Type_fr_link3_X_fr_link2 : public TransformForce<SCALAR, Type_fr_link3_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link3_X_fr_link2();
        const Type_fr_link3_X_fr_link2& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_link3 : public TransformForce<SCALAR, Type_fr_link2_X_fr_link3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_link3();
        const Type_fr_link2_X_fr_link3& update(const JState&);
    protected:
    };
    
    class Type_fr_link4_X_fr_link3 : public TransformForce<SCALAR, Type_fr_link4_X_fr_link3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link4_X_fr_link3();
        const Type_fr_link4_X_fr_link3& update(const JState&);
    protected:
    };
    
    class Type_fr_link3_X_fr_link4 : public TransformForce<SCALAR, Type_fr_link3_X_fr_link4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link3_X_fr_link4();
        const Type_fr_link3_X_fr_link4& update(const JState&);
    protected:
    };
    
    class Type_fr_link5_X_fr_link4 : public TransformForce<SCALAR, Type_fr_link5_X_fr_link4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link5_X_fr_link4();
        const Type_fr_link5_X_fr_link4& update(const JState&);
    protected:
    };
    
    class Type_fr_link4_X_fr_link5 : public TransformForce<SCALAR, Type_fr_link4_X_fr_link5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link4_X_fr_link5();
        const Type_fr_link4_X_fr_link5& update(const JState&);
    protected:
    };
    
    class Type_fr_link6_X_fr_link5 : public TransformForce<SCALAR, Type_fr_link6_X_fr_link5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link6_X_fr_link5();
        const Type_fr_link6_X_fr_link5& update(const JState&);
    protected:
    };
    
    class Type_fr_link5_X_fr_link6 : public TransformForce<SCALAR, Type_fr_link5_X_fr_link6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link5_X_fr_link6();
        const Type_fr_link5_X_fr_link6& update(const JState&);
    protected:
    };
    
public:
    ForceTransforms();
    void updateParameters();
    Type_fr_link0_X_fr_link1 fr_link0_X_fr_link1;
    Type_fr_link0_X_fr_link2 fr_link0_X_fr_link2;
    Type_fr_link0_X_fr_link3 fr_link0_X_fr_link3;
    Type_fr_link0_X_fr_link4 fr_link0_X_fr_link4;
    Type_fr_link0_X_fr_link5 fr_link0_X_fr_link5;
    Type_fr_link0_X_fr_link6 fr_link0_X_fr_link6;
    Type_fr_link0_X_fr_ee fr_link0_X_fr_ee;
    Type_fr_link0_X_com0 fr_link0_X_com0;
    Type_fr_link0_X_com1 fr_link0_X_com1;
    Type_fr_link0_X_com2 fr_link0_X_com2;
    Type_fr_link0_X_com3 fr_link0_X_com3;
    Type_fr_link0_X_com4 fr_link0_X_com4;
    Type_fr_link0_X_com5 fr_link0_X_com5;
    Type_fr_link0_X_com6 fr_link0_X_com6;
    Type_fr_link1_X_fr_link0 fr_link1_X_fr_link0;
    Type_fr_link2_X_fr_link0 fr_link2_X_fr_link0;
    Type_fr_link3_X_fr_link0 fr_link3_X_fr_link0;
    Type_fr_link4_X_fr_link0 fr_link4_X_fr_link0;
    Type_fr_link5_X_fr_link0 fr_link5_X_fr_link0;
    Type_fr_link6_X_fr_link0 fr_link6_X_fr_link0;
    Type_fr_ee_X_fr_link0 fr_ee_X_fr_link0;
    Type_com0_X_fr_link0 com0_X_fr_link0;
    Type_com1_X_fr_link0 com1_X_fr_link0;
    Type_com2_X_fr_link0 com2_X_fr_link0;
    Type_com3_X_fr_link0 com3_X_fr_link0;
    Type_com4_X_fr_link0 com4_X_fr_link0;
    Type_com5_X_fr_link0 com5_X_fr_link0;
    Type_com6_X_fr_link0 com6_X_fr_link0;
    Type_fr_link0_X_fr_jA fr_link0_X_fr_jA;
    Type_fr_link0_X_fr_jB fr_link0_X_fr_jB;
    Type_fr_link0_X_fr_jC fr_link0_X_fr_jC;
    Type_fr_link0_X_fr_jD fr_link0_X_fr_jD;
    Type_fr_link0_X_fr_jE fr_link0_X_fr_jE;
    Type_fr_link0_X_fr_jF fr_link0_X_fr_jF;
    Type_fr_link2_X_fr_link1 fr_link2_X_fr_link1;
    Type_fr_link1_X_fr_link2 fr_link1_X_fr_link2;
    Type_fr_link3_X_fr_link2 fr_link3_X_fr_link2;
    Type_fr_link2_X_fr_link3 fr_link2_X_fr_link3;
    Type_fr_link4_X_fr_link3 fr_link4_X_fr_link3;
    Type_fr_link3_X_fr_link4 fr_link3_X_fr_link4;
    Type_fr_link5_X_fr_link4 fr_link5_X_fr_link4;
    Type_fr_link4_X_fr_link5 fr_link4_X_fr_link5;
    Type_fr_link6_X_fr_link5 fr_link6_X_fr_link5;
    Type_fr_link5_X_fr_link6 fr_link5_X_fr_link6;

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
    class Type_fr_link0_X_fr_link1 : public TransformHomogeneous<SCALAR, Type_fr_link0_X_fr_link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_link1();
        const Type_fr_link0_X_fr_link1& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_link2 : public TransformHomogeneous<SCALAR, Type_fr_link0_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_link2();
        const Type_fr_link0_X_fr_link2& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_link3 : public TransformHomogeneous<SCALAR, Type_fr_link0_X_fr_link3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_link3();
        const Type_fr_link0_X_fr_link3& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_link4 : public TransformHomogeneous<SCALAR, Type_fr_link0_X_fr_link4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_link4();
        const Type_fr_link0_X_fr_link4& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_link5 : public TransformHomogeneous<SCALAR, Type_fr_link0_X_fr_link5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_link5();
        const Type_fr_link0_X_fr_link5& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_link6 : public TransformHomogeneous<SCALAR, Type_fr_link0_X_fr_link6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_link6();
        const Type_fr_link0_X_fr_link6& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_ee : public TransformHomogeneous<SCALAR, Type_fr_link0_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_ee();
        const Type_fr_link0_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com0 : public TransformHomogeneous<SCALAR, Type_fr_link0_X_com0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com0();
        const Type_fr_link0_X_com0& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com1 : public TransformHomogeneous<SCALAR, Type_fr_link0_X_com1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com1();
        const Type_fr_link0_X_com1& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com2 : public TransformHomogeneous<SCALAR, Type_fr_link0_X_com2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com2();
        const Type_fr_link0_X_com2& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com3 : public TransformHomogeneous<SCALAR, Type_fr_link0_X_com3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com3();
        const Type_fr_link0_X_com3& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com4 : public TransformHomogeneous<SCALAR, Type_fr_link0_X_com4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com4();
        const Type_fr_link0_X_com4& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com5 : public TransformHomogeneous<SCALAR, Type_fr_link0_X_com5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com5();
        const Type_fr_link0_X_com5& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_com6 : public TransformHomogeneous<SCALAR, Type_fr_link0_X_com6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_com6();
        const Type_fr_link0_X_com6& update(const JState&);
    protected:
    };
    
    class Type_fr_link1_X_fr_link0 : public TransformHomogeneous<SCALAR, Type_fr_link1_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link1_X_fr_link0();
        const Type_fr_link1_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_link0 : public TransformHomogeneous<SCALAR, Type_fr_link2_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_link0();
        const Type_fr_link2_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_link3_X_fr_link0 : public TransformHomogeneous<SCALAR, Type_fr_link3_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link3_X_fr_link0();
        const Type_fr_link3_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_link4_X_fr_link0 : public TransformHomogeneous<SCALAR, Type_fr_link4_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link4_X_fr_link0();
        const Type_fr_link4_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_link5_X_fr_link0 : public TransformHomogeneous<SCALAR, Type_fr_link5_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link5_X_fr_link0();
        const Type_fr_link5_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_link6_X_fr_link0 : public TransformHomogeneous<SCALAR, Type_fr_link6_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link6_X_fr_link0();
        const Type_fr_link6_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_ee_X_fr_link0 : public TransformHomogeneous<SCALAR, Type_fr_ee_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ee_X_fr_link0();
        const Type_fr_ee_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com0_X_fr_link0 : public TransformHomogeneous<SCALAR, Type_com0_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com0_X_fr_link0();
        const Type_com0_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com1_X_fr_link0 : public TransformHomogeneous<SCALAR, Type_com1_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com1_X_fr_link0();
        const Type_com1_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com2_X_fr_link0 : public TransformHomogeneous<SCALAR, Type_com2_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com2_X_fr_link0();
        const Type_com2_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com3_X_fr_link0 : public TransformHomogeneous<SCALAR, Type_com3_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com3_X_fr_link0();
        const Type_com3_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com4_X_fr_link0 : public TransformHomogeneous<SCALAR, Type_com4_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com4_X_fr_link0();
        const Type_com4_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com5_X_fr_link0 : public TransformHomogeneous<SCALAR, Type_com5_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com5_X_fr_link0();
        const Type_com5_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_com6_X_fr_link0 : public TransformHomogeneous<SCALAR, Type_com6_X_fr_link0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_com6_X_fr_link0();
        const Type_com6_X_fr_link0& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_jA : public TransformHomogeneous<SCALAR, Type_fr_link0_X_fr_jA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_jA();
        const Type_fr_link0_X_fr_jA& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_jB : public TransformHomogeneous<SCALAR, Type_fr_link0_X_fr_jB>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_jB();
        const Type_fr_link0_X_fr_jB& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_jC : public TransformHomogeneous<SCALAR, Type_fr_link0_X_fr_jC>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_jC();
        const Type_fr_link0_X_fr_jC& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_jD : public TransformHomogeneous<SCALAR, Type_fr_link0_X_fr_jD>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_jD();
        const Type_fr_link0_X_fr_jD& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_jE : public TransformHomogeneous<SCALAR, Type_fr_link0_X_fr_jE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_jE();
        const Type_fr_link0_X_fr_jE& update(const JState&);
    protected:
    };
    
    class Type_fr_link0_X_fr_jF : public TransformHomogeneous<SCALAR, Type_fr_link0_X_fr_jF>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link0_X_fr_jF();
        const Type_fr_link0_X_fr_jF& update(const JState&);
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
    
    class Type_fr_link3_X_fr_link2 : public TransformHomogeneous<SCALAR, Type_fr_link3_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link3_X_fr_link2();
        const Type_fr_link3_X_fr_link2& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_link3 : public TransformHomogeneous<SCALAR, Type_fr_link2_X_fr_link3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_link3();
        const Type_fr_link2_X_fr_link3& update(const JState&);
    protected:
    };
    
    class Type_fr_link4_X_fr_link3 : public TransformHomogeneous<SCALAR, Type_fr_link4_X_fr_link3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link4_X_fr_link3();
        const Type_fr_link4_X_fr_link3& update(const JState&);
    protected:
    };
    
    class Type_fr_link3_X_fr_link4 : public TransformHomogeneous<SCALAR, Type_fr_link3_X_fr_link4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link3_X_fr_link4();
        const Type_fr_link3_X_fr_link4& update(const JState&);
    protected:
    };
    
    class Type_fr_link5_X_fr_link4 : public TransformHomogeneous<SCALAR, Type_fr_link5_X_fr_link4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link5_X_fr_link4();
        const Type_fr_link5_X_fr_link4& update(const JState&);
    protected:
    };
    
    class Type_fr_link4_X_fr_link5 : public TransformHomogeneous<SCALAR, Type_fr_link4_X_fr_link5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link4_X_fr_link5();
        const Type_fr_link4_X_fr_link5& update(const JState&);
    protected:
    };
    
    class Type_fr_link6_X_fr_link5 : public TransformHomogeneous<SCALAR, Type_fr_link6_X_fr_link5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link6_X_fr_link5();
        const Type_fr_link6_X_fr_link5& update(const JState&);
    protected:
    };
    
    class Type_fr_link5_X_fr_link6 : public TransformHomogeneous<SCALAR, Type_fr_link5_X_fr_link6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link5_X_fr_link6();
        const Type_fr_link5_X_fr_link6& update(const JState&);
    protected:
    };
    
public:
    HomogeneousTransforms();
    void updateParameters();
    Type_fr_link0_X_fr_link1 fr_link0_X_fr_link1;
    Type_fr_link0_X_fr_link2 fr_link0_X_fr_link2;
    Type_fr_link0_X_fr_link3 fr_link0_X_fr_link3;
    Type_fr_link0_X_fr_link4 fr_link0_X_fr_link4;
    Type_fr_link0_X_fr_link5 fr_link0_X_fr_link5;
    Type_fr_link0_X_fr_link6 fr_link0_X_fr_link6;
    Type_fr_link0_X_fr_ee fr_link0_X_fr_ee;
    Type_fr_link0_X_com0 fr_link0_X_com0;
    Type_fr_link0_X_com1 fr_link0_X_com1;
    Type_fr_link0_X_com2 fr_link0_X_com2;
    Type_fr_link0_X_com3 fr_link0_X_com3;
    Type_fr_link0_X_com4 fr_link0_X_com4;
    Type_fr_link0_X_com5 fr_link0_X_com5;
    Type_fr_link0_X_com6 fr_link0_X_com6;
    Type_fr_link1_X_fr_link0 fr_link1_X_fr_link0;
    Type_fr_link2_X_fr_link0 fr_link2_X_fr_link0;
    Type_fr_link3_X_fr_link0 fr_link3_X_fr_link0;
    Type_fr_link4_X_fr_link0 fr_link4_X_fr_link0;
    Type_fr_link5_X_fr_link0 fr_link5_X_fr_link0;
    Type_fr_link6_X_fr_link0 fr_link6_X_fr_link0;
    Type_fr_ee_X_fr_link0 fr_ee_X_fr_link0;
    Type_com0_X_fr_link0 com0_X_fr_link0;
    Type_com1_X_fr_link0 com1_X_fr_link0;
    Type_com2_X_fr_link0 com2_X_fr_link0;
    Type_com3_X_fr_link0 com3_X_fr_link0;
    Type_com4_X_fr_link0 com4_X_fr_link0;
    Type_com5_X_fr_link0 com5_X_fr_link0;
    Type_com6_X_fr_link0 com6_X_fr_link0;
    Type_fr_link0_X_fr_jA fr_link0_X_fr_jA;
    Type_fr_link0_X_fr_jB fr_link0_X_fr_jB;
    Type_fr_link0_X_fr_jC fr_link0_X_fr_jC;
    Type_fr_link0_X_fr_jD fr_link0_X_fr_jD;
    Type_fr_link0_X_fr_jE fr_link0_X_fr_jE;
    Type_fr_link0_X_fr_jF fr_link0_X_fr_jF;
    Type_fr_link2_X_fr_link1 fr_link2_X_fr_link1;
    Type_fr_link1_X_fr_link2 fr_link1_X_fr_link2;
    Type_fr_link3_X_fr_link2 fr_link3_X_fr_link2;
    Type_fr_link2_X_fr_link3 fr_link2_X_fr_link3;
    Type_fr_link4_X_fr_link3 fr_link4_X_fr_link3;
    Type_fr_link3_X_fr_link4 fr_link3_X_fr_link4;
    Type_fr_link5_X_fr_link4 fr_link5_X_fr_link4;
    Type_fr_link4_X_fr_link5 fr_link4_X_fr_link5;
    Type_fr_link6_X_fr_link5 fr_link6_X_fr_link5;
    Type_fr_link5_X_fr_link6 fr_link5_X_fr_link6;

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
