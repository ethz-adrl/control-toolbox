#ifndef HYQ_TRANSFORMS_H_
#define HYQ_TRANSFORMS_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include <iit/rbd/traits/DoubleTrait.h>
#include "kinematics_parameters.h"

namespace iit {
namespace HyQ {

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
    class Type_fr_trunk_X_fr_LF_hipassembly : public TransformMotion<Scalar, Type_fr_trunk_X_fr_LF_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_hipassembly();
        const Type_fr_trunk_X_fr_LF_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_hipassembly : public TransformMotion<Scalar, Type_fr_trunk_X_fr_RF_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_hipassembly();
        const Type_fr_trunk_X_fr_RF_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_hipassembly : public TransformMotion<Scalar, Type_fr_trunk_X_fr_LH_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_hipassembly();
        const Type_fr_trunk_X_fr_LH_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_hipassembly : public TransformMotion<Scalar, Type_fr_trunk_X_fr_RH_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_hipassembly();
        const Type_fr_trunk_X_fr_RH_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_upperleg : public TransformMotion<Scalar, Type_fr_trunk_X_fr_LF_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_upperleg();
        const Type_fr_trunk_X_fr_LF_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_upperleg : public TransformMotion<Scalar, Type_fr_trunk_X_fr_RF_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_upperleg();
        const Type_fr_trunk_X_fr_RF_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_upperleg : public TransformMotion<Scalar, Type_fr_trunk_X_fr_LH_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_upperleg();
        const Type_fr_trunk_X_fr_LH_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_upperleg : public TransformMotion<Scalar, Type_fr_trunk_X_fr_RH_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_upperleg();
        const Type_fr_trunk_X_fr_RH_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_lowerleg : public TransformMotion<Scalar, Type_fr_trunk_X_fr_LF_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_lowerleg();
        const Type_fr_trunk_X_fr_LF_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_lowerleg : public TransformMotion<Scalar, Type_fr_trunk_X_fr_RF_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_lowerleg();
        const Type_fr_trunk_X_fr_RF_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_lowerleg : public TransformMotion<Scalar, Type_fr_trunk_X_fr_LH_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_lowerleg();
        const Type_fr_trunk_X_fr_LH_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_lowerleg : public TransformMotion<Scalar, Type_fr_trunk_X_fr_RH_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_lowerleg();
        const Type_fr_trunk_X_fr_RH_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_hipassembly_X_fr_trunk : public TransformMotion<Scalar, Type_fr_LF_hipassembly_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_hipassembly_X_fr_trunk();
        const Type_fr_LF_hipassembly_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_hipassembly_X_fr_trunk : public TransformMotion<Scalar, Type_fr_RF_hipassembly_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_hipassembly_X_fr_trunk();
        const Type_fr_RF_hipassembly_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_hipassembly_X_fr_trunk : public TransformMotion<Scalar, Type_fr_LH_hipassembly_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_hipassembly_X_fr_trunk();
        const Type_fr_LH_hipassembly_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_hipassembly_X_fr_trunk : public TransformMotion<Scalar, Type_fr_RH_hipassembly_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_hipassembly_X_fr_trunk();
        const Type_fr_RH_hipassembly_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_upperleg_X_fr_trunk : public TransformMotion<Scalar, Type_fr_LF_upperleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_upperleg_X_fr_trunk();
        const Type_fr_LF_upperleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_upperleg_X_fr_trunk : public TransformMotion<Scalar, Type_fr_RF_upperleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_upperleg_X_fr_trunk();
        const Type_fr_RF_upperleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_upperleg_X_fr_trunk : public TransformMotion<Scalar, Type_fr_LH_upperleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_upperleg_X_fr_trunk();
        const Type_fr_LH_upperleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_upperleg_X_fr_trunk : public TransformMotion<Scalar, Type_fr_RH_upperleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_upperleg_X_fr_trunk();
        const Type_fr_RH_upperleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_lowerleg_X_fr_trunk : public TransformMotion<Scalar, Type_fr_LF_lowerleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_lowerleg_X_fr_trunk();
        const Type_fr_LF_lowerleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_lowerleg_X_fr_trunk : public TransformMotion<Scalar, Type_fr_RF_lowerleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_lowerleg_X_fr_trunk();
        const Type_fr_RF_lowerleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_lowerleg_X_fr_trunk : public TransformMotion<Scalar, Type_fr_LH_lowerleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_lowerleg_X_fr_trunk();
        const Type_fr_LH_lowerleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_lowerleg_X_fr_trunk : public TransformMotion<Scalar, Type_fr_RH_lowerleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_lowerleg_X_fr_trunk();
        const Type_fr_RH_lowerleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_LF_hipassemblyCOM : public TransformMotion<Scalar, Type_fr_trunk_X_LF_hipassemblyCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_LF_hipassemblyCOM();
        const Type_fr_trunk_X_LF_hipassemblyCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_hipassemblyCOM : public TransformMotion<Scalar, Type_fr_trunk_X_RF_hipassemblyCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_RF_hipassemblyCOM();
        const Type_fr_trunk_X_RF_hipassemblyCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_LH_hipassemblyCOM : public TransformMotion<Scalar, Type_fr_trunk_X_LH_hipassemblyCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_LH_hipassemblyCOM();
        const Type_fr_trunk_X_LH_hipassemblyCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_RH_hipassemblyCOM : public TransformMotion<Scalar, Type_fr_trunk_X_RH_hipassemblyCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_RH_hipassemblyCOM();
        const Type_fr_trunk_X_RH_hipassemblyCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_LF_upperlegCOM : public TransformMotion<Scalar, Type_fr_trunk_X_LF_upperlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_LF_upperlegCOM();
        const Type_fr_trunk_X_LF_upperlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_upperlegCOM : public TransformMotion<Scalar, Type_fr_trunk_X_RF_upperlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_RF_upperlegCOM();
        const Type_fr_trunk_X_RF_upperlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_LH_upperlegCOM : public TransformMotion<Scalar, Type_fr_trunk_X_LH_upperlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_LH_upperlegCOM();
        const Type_fr_trunk_X_LH_upperlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_RH_upperlegCOM : public TransformMotion<Scalar, Type_fr_trunk_X_RH_upperlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_RH_upperlegCOM();
        const Type_fr_trunk_X_RH_upperlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_LF_lowerlegCOM : public TransformMotion<Scalar, Type_fr_trunk_X_LF_lowerlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_LF_lowerlegCOM();
        const Type_fr_trunk_X_LF_lowerlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_lowerlegCOM : public TransformMotion<Scalar, Type_fr_trunk_X_RF_lowerlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_RF_lowerlegCOM();
        const Type_fr_trunk_X_RF_lowerlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_LH_lowerlegCOM : public TransformMotion<Scalar, Type_fr_trunk_X_LH_lowerlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_LH_lowerlegCOM();
        const Type_fr_trunk_X_LH_lowerlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_RH_lowerlegCOM : public TransformMotion<Scalar, Type_fr_trunk_X_RH_lowerlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_RH_lowerlegCOM();
        const Type_fr_trunk_X_RH_lowerlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_foot_X_fr_LF_lowerleg : public TransformMotion<Scalar, Type_fr_LF_foot_X_fr_LF_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_foot_X_fr_LF_lowerleg();
        const Type_fr_LF_foot_X_fr_LF_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_foot_X_fr_RF_lowerleg : public TransformMotion<Scalar, Type_fr_RF_foot_X_fr_RF_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_foot_X_fr_RF_lowerleg();
        const Type_fr_RF_foot_X_fr_RF_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_foot_X_fr_LH_lowerleg : public TransformMotion<Scalar, Type_fr_LH_foot_X_fr_LH_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_foot_X_fr_LH_lowerleg();
        const Type_fr_LH_foot_X_fr_LH_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_foot_X_fr_RH_lowerleg : public TransformMotion<Scalar, Type_fr_RH_foot_X_fr_RH_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_foot_X_fr_RH_lowerleg();
        const Type_fr_RH_foot_X_fr_RH_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_foot : public TransformMotion<Scalar, Type_fr_trunk_X_fr_LF_foot>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_foot();
        const Type_fr_trunk_X_fr_LF_foot& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_foot : public TransformMotion<Scalar, Type_fr_trunk_X_fr_RF_foot>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_foot();
        const Type_fr_trunk_X_fr_RF_foot& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_foot : public TransformMotion<Scalar, Type_fr_trunk_X_fr_LH_foot>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_foot();
        const Type_fr_trunk_X_fr_LH_foot& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_foot : public TransformMotion<Scalar, Type_fr_trunk_X_fr_RH_foot>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_foot();
        const Type_fr_trunk_X_fr_RH_foot& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_foot_X_fr_trunk : public TransformMotion<Scalar, Type_fr_LF_foot_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_foot_X_fr_trunk();
        const Type_fr_LF_foot_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_foot_X_fr_trunk : public TransformMotion<Scalar, Type_fr_RF_foot_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_foot_X_fr_trunk();
        const Type_fr_RF_foot_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_foot_X_fr_trunk : public TransformMotion<Scalar, Type_fr_LH_foot_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_foot_X_fr_trunk();
        const Type_fr_LH_foot_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_foot_X_fr_trunk : public TransformMotion<Scalar, Type_fr_RH_foot_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_foot_X_fr_trunk();
        const Type_fr_RH_foot_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_HAA : public TransformMotion<Scalar, Type_fr_trunk_X_fr_LF_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_HAA();
        const Type_fr_trunk_X_fr_LF_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_HAA : public TransformMotion<Scalar, Type_fr_trunk_X_fr_RF_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_HAA();
        const Type_fr_trunk_X_fr_RF_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_HAA : public TransformMotion<Scalar, Type_fr_trunk_X_fr_LH_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_HAA();
        const Type_fr_trunk_X_fr_LH_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_HAA : public TransformMotion<Scalar, Type_fr_trunk_X_fr_RH_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_HAA();
        const Type_fr_trunk_X_fr_RH_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_HFE : public TransformMotion<Scalar, Type_fr_trunk_X_fr_LF_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_HFE();
        const Type_fr_trunk_X_fr_LF_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_HFE : public TransformMotion<Scalar, Type_fr_trunk_X_fr_RF_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_HFE();
        const Type_fr_trunk_X_fr_RF_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_HFE : public TransformMotion<Scalar, Type_fr_trunk_X_fr_LH_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_HFE();
        const Type_fr_trunk_X_fr_LH_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_HFE : public TransformMotion<Scalar, Type_fr_trunk_X_fr_RH_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_HFE();
        const Type_fr_trunk_X_fr_RH_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_KFE : public TransformMotion<Scalar, Type_fr_trunk_X_fr_LF_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_KFE();
        const Type_fr_trunk_X_fr_LF_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_KFE : public TransformMotion<Scalar, Type_fr_trunk_X_fr_RF_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_KFE();
        const Type_fr_trunk_X_fr_RF_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_KFE : public TransformMotion<Scalar, Type_fr_trunk_X_fr_LH_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_KFE();
        const Type_fr_trunk_X_fr_LH_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_KFE : public TransformMotion<Scalar, Type_fr_trunk_X_fr_RH_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_KFE();
        const Type_fr_trunk_X_fr_RH_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_upperleg_X_fr_LF_hipassembly : public TransformMotion<Scalar, Type_fr_LF_upperleg_X_fr_LF_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_upperleg_X_fr_LF_hipassembly();
        const Type_fr_LF_upperleg_X_fr_LF_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_hipassembly_X_fr_LF_upperleg : public TransformMotion<Scalar, Type_fr_LF_hipassembly_X_fr_LF_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_hipassembly_X_fr_LF_upperleg();
        const Type_fr_LF_hipassembly_X_fr_LF_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_lowerleg_X_fr_LF_upperleg : public TransformMotion<Scalar, Type_fr_LF_lowerleg_X_fr_LF_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_lowerleg_X_fr_LF_upperleg();
        const Type_fr_LF_lowerleg_X_fr_LF_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_upperleg_X_fr_LF_lowerleg : public TransformMotion<Scalar, Type_fr_LF_upperleg_X_fr_LF_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_upperleg_X_fr_LF_lowerleg();
        const Type_fr_LF_upperleg_X_fr_LF_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_upperleg_X_fr_RF_hipassembly : public TransformMotion<Scalar, Type_fr_RF_upperleg_X_fr_RF_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_upperleg_X_fr_RF_hipassembly();
        const Type_fr_RF_upperleg_X_fr_RF_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_hipassembly_X_fr_RF_upperleg : public TransformMotion<Scalar, Type_fr_RF_hipassembly_X_fr_RF_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_hipassembly_X_fr_RF_upperleg();
        const Type_fr_RF_hipassembly_X_fr_RF_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_lowerleg_X_fr_RF_upperleg : public TransformMotion<Scalar, Type_fr_RF_lowerleg_X_fr_RF_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_lowerleg_X_fr_RF_upperleg();
        const Type_fr_RF_lowerleg_X_fr_RF_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_upperleg_X_fr_RF_lowerleg : public TransformMotion<Scalar, Type_fr_RF_upperleg_X_fr_RF_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_upperleg_X_fr_RF_lowerleg();
        const Type_fr_RF_upperleg_X_fr_RF_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_upperleg_X_fr_LH_hipassembly : public TransformMotion<Scalar, Type_fr_LH_upperleg_X_fr_LH_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_upperleg_X_fr_LH_hipassembly();
        const Type_fr_LH_upperleg_X_fr_LH_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_hipassembly_X_fr_LH_upperleg : public TransformMotion<Scalar, Type_fr_LH_hipassembly_X_fr_LH_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_hipassembly_X_fr_LH_upperleg();
        const Type_fr_LH_hipassembly_X_fr_LH_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_lowerleg_X_fr_LH_upperleg : public TransformMotion<Scalar, Type_fr_LH_lowerleg_X_fr_LH_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_lowerleg_X_fr_LH_upperleg();
        const Type_fr_LH_lowerleg_X_fr_LH_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_upperleg_X_fr_LH_lowerleg : public TransformMotion<Scalar, Type_fr_LH_upperleg_X_fr_LH_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_upperleg_X_fr_LH_lowerleg();
        const Type_fr_LH_upperleg_X_fr_LH_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_upperleg_X_fr_RH_hipassembly : public TransformMotion<Scalar, Type_fr_RH_upperleg_X_fr_RH_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_upperleg_X_fr_RH_hipassembly();
        const Type_fr_RH_upperleg_X_fr_RH_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_hipassembly_X_fr_RH_upperleg : public TransformMotion<Scalar, Type_fr_RH_hipassembly_X_fr_RH_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_hipassembly_X_fr_RH_upperleg();
        const Type_fr_RH_hipassembly_X_fr_RH_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_lowerleg_X_fr_RH_upperleg : public TransformMotion<Scalar, Type_fr_RH_lowerleg_X_fr_RH_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_lowerleg_X_fr_RH_upperleg();
        const Type_fr_RH_lowerleg_X_fr_RH_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_upperleg_X_fr_RH_lowerleg : public TransformMotion<Scalar, Type_fr_RH_upperleg_X_fr_RH_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_upperleg_X_fr_RH_lowerleg();
        const Type_fr_RH_upperleg_X_fr_RH_lowerleg& update(const JState&);
    protected:
    };
    
public:
    MotionTransforms();
    void updateParameters();
    Type_fr_trunk_X_fr_LF_hipassembly fr_trunk_X_fr_LF_hipassembly;
    Type_fr_trunk_X_fr_RF_hipassembly fr_trunk_X_fr_RF_hipassembly;
    Type_fr_trunk_X_fr_LH_hipassembly fr_trunk_X_fr_LH_hipassembly;
    Type_fr_trunk_X_fr_RH_hipassembly fr_trunk_X_fr_RH_hipassembly;
    Type_fr_trunk_X_fr_LF_upperleg fr_trunk_X_fr_LF_upperleg;
    Type_fr_trunk_X_fr_RF_upperleg fr_trunk_X_fr_RF_upperleg;
    Type_fr_trunk_X_fr_LH_upperleg fr_trunk_X_fr_LH_upperleg;
    Type_fr_trunk_X_fr_RH_upperleg fr_trunk_X_fr_RH_upperleg;
    Type_fr_trunk_X_fr_LF_lowerleg fr_trunk_X_fr_LF_lowerleg;
    Type_fr_trunk_X_fr_RF_lowerleg fr_trunk_X_fr_RF_lowerleg;
    Type_fr_trunk_X_fr_LH_lowerleg fr_trunk_X_fr_LH_lowerleg;
    Type_fr_trunk_X_fr_RH_lowerleg fr_trunk_X_fr_RH_lowerleg;
    Type_fr_LF_hipassembly_X_fr_trunk fr_LF_hipassembly_X_fr_trunk;
    Type_fr_RF_hipassembly_X_fr_trunk fr_RF_hipassembly_X_fr_trunk;
    Type_fr_LH_hipassembly_X_fr_trunk fr_LH_hipassembly_X_fr_trunk;
    Type_fr_RH_hipassembly_X_fr_trunk fr_RH_hipassembly_X_fr_trunk;
    Type_fr_LF_upperleg_X_fr_trunk fr_LF_upperleg_X_fr_trunk;
    Type_fr_RF_upperleg_X_fr_trunk fr_RF_upperleg_X_fr_trunk;
    Type_fr_LH_upperleg_X_fr_trunk fr_LH_upperleg_X_fr_trunk;
    Type_fr_RH_upperleg_X_fr_trunk fr_RH_upperleg_X_fr_trunk;
    Type_fr_LF_lowerleg_X_fr_trunk fr_LF_lowerleg_X_fr_trunk;
    Type_fr_RF_lowerleg_X_fr_trunk fr_RF_lowerleg_X_fr_trunk;
    Type_fr_LH_lowerleg_X_fr_trunk fr_LH_lowerleg_X_fr_trunk;
    Type_fr_RH_lowerleg_X_fr_trunk fr_RH_lowerleg_X_fr_trunk;
    Type_fr_trunk_X_LF_hipassemblyCOM fr_trunk_X_LF_hipassemblyCOM;
    Type_fr_trunk_X_RF_hipassemblyCOM fr_trunk_X_RF_hipassemblyCOM;
    Type_fr_trunk_X_LH_hipassemblyCOM fr_trunk_X_LH_hipassemblyCOM;
    Type_fr_trunk_X_RH_hipassemblyCOM fr_trunk_X_RH_hipassemblyCOM;
    Type_fr_trunk_X_LF_upperlegCOM fr_trunk_X_LF_upperlegCOM;
    Type_fr_trunk_X_RF_upperlegCOM fr_trunk_X_RF_upperlegCOM;
    Type_fr_trunk_X_LH_upperlegCOM fr_trunk_X_LH_upperlegCOM;
    Type_fr_trunk_X_RH_upperlegCOM fr_trunk_X_RH_upperlegCOM;
    Type_fr_trunk_X_LF_lowerlegCOM fr_trunk_X_LF_lowerlegCOM;
    Type_fr_trunk_X_RF_lowerlegCOM fr_trunk_X_RF_lowerlegCOM;
    Type_fr_trunk_X_LH_lowerlegCOM fr_trunk_X_LH_lowerlegCOM;
    Type_fr_trunk_X_RH_lowerlegCOM fr_trunk_X_RH_lowerlegCOM;
    Type_fr_LF_foot_X_fr_LF_lowerleg fr_LF_foot_X_fr_LF_lowerleg;
    Type_fr_RF_foot_X_fr_RF_lowerleg fr_RF_foot_X_fr_RF_lowerleg;
    Type_fr_LH_foot_X_fr_LH_lowerleg fr_LH_foot_X_fr_LH_lowerleg;
    Type_fr_RH_foot_X_fr_RH_lowerleg fr_RH_foot_X_fr_RH_lowerleg;
    Type_fr_trunk_X_fr_LF_foot fr_trunk_X_fr_LF_foot;
    Type_fr_trunk_X_fr_RF_foot fr_trunk_X_fr_RF_foot;
    Type_fr_trunk_X_fr_LH_foot fr_trunk_X_fr_LH_foot;
    Type_fr_trunk_X_fr_RH_foot fr_trunk_X_fr_RH_foot;
    Type_fr_LF_foot_X_fr_trunk fr_LF_foot_X_fr_trunk;
    Type_fr_RF_foot_X_fr_trunk fr_RF_foot_X_fr_trunk;
    Type_fr_LH_foot_X_fr_trunk fr_LH_foot_X_fr_trunk;
    Type_fr_RH_foot_X_fr_trunk fr_RH_foot_X_fr_trunk;
    Type_fr_trunk_X_fr_LF_HAA fr_trunk_X_fr_LF_HAA;
    Type_fr_trunk_X_fr_RF_HAA fr_trunk_X_fr_RF_HAA;
    Type_fr_trunk_X_fr_LH_HAA fr_trunk_X_fr_LH_HAA;
    Type_fr_trunk_X_fr_RH_HAA fr_trunk_X_fr_RH_HAA;
    Type_fr_trunk_X_fr_LF_HFE fr_trunk_X_fr_LF_HFE;
    Type_fr_trunk_X_fr_RF_HFE fr_trunk_X_fr_RF_HFE;
    Type_fr_trunk_X_fr_LH_HFE fr_trunk_X_fr_LH_HFE;
    Type_fr_trunk_X_fr_RH_HFE fr_trunk_X_fr_RH_HFE;
    Type_fr_trunk_X_fr_LF_KFE fr_trunk_X_fr_LF_KFE;
    Type_fr_trunk_X_fr_RF_KFE fr_trunk_X_fr_RF_KFE;
    Type_fr_trunk_X_fr_LH_KFE fr_trunk_X_fr_LH_KFE;
    Type_fr_trunk_X_fr_RH_KFE fr_trunk_X_fr_RH_KFE;
    Type_fr_LF_upperleg_X_fr_LF_hipassembly fr_LF_upperleg_X_fr_LF_hipassembly;
    Type_fr_LF_hipassembly_X_fr_LF_upperleg fr_LF_hipassembly_X_fr_LF_upperleg;
    Type_fr_LF_lowerleg_X_fr_LF_upperleg fr_LF_lowerleg_X_fr_LF_upperleg;
    Type_fr_LF_upperleg_X_fr_LF_lowerleg fr_LF_upperleg_X_fr_LF_lowerleg;
    Type_fr_RF_upperleg_X_fr_RF_hipassembly fr_RF_upperleg_X_fr_RF_hipassembly;
    Type_fr_RF_hipassembly_X_fr_RF_upperleg fr_RF_hipassembly_X_fr_RF_upperleg;
    Type_fr_RF_lowerleg_X_fr_RF_upperleg fr_RF_lowerleg_X_fr_RF_upperleg;
    Type_fr_RF_upperleg_X_fr_RF_lowerleg fr_RF_upperleg_X_fr_RF_lowerleg;
    Type_fr_LH_upperleg_X_fr_LH_hipassembly fr_LH_upperleg_X_fr_LH_hipassembly;
    Type_fr_LH_hipassembly_X_fr_LH_upperleg fr_LH_hipassembly_X_fr_LH_upperleg;
    Type_fr_LH_lowerleg_X_fr_LH_upperleg fr_LH_lowerleg_X_fr_LH_upperleg;
    Type_fr_LH_upperleg_X_fr_LH_lowerleg fr_LH_upperleg_X_fr_LH_lowerleg;
    Type_fr_RH_upperleg_X_fr_RH_hipassembly fr_RH_upperleg_X_fr_RH_hipassembly;
    Type_fr_RH_hipassembly_X_fr_RH_upperleg fr_RH_hipassembly_X_fr_RH_upperleg;
    Type_fr_RH_lowerleg_X_fr_RH_upperleg fr_RH_lowerleg_X_fr_RH_upperleg;
    Type_fr_RH_upperleg_X_fr_RH_lowerleg fr_RH_upperleg_X_fr_RH_lowerleg;

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
    class Type_fr_trunk_X_fr_LF_hipassembly : public TransformForce<Scalar, Type_fr_trunk_X_fr_LF_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_hipassembly();
        const Type_fr_trunk_X_fr_LF_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_hipassembly : public TransformForce<Scalar, Type_fr_trunk_X_fr_RF_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_hipassembly();
        const Type_fr_trunk_X_fr_RF_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_hipassembly : public TransformForce<Scalar, Type_fr_trunk_X_fr_LH_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_hipassembly();
        const Type_fr_trunk_X_fr_LH_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_hipassembly : public TransformForce<Scalar, Type_fr_trunk_X_fr_RH_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_hipassembly();
        const Type_fr_trunk_X_fr_RH_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_upperleg : public TransformForce<Scalar, Type_fr_trunk_X_fr_LF_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_upperleg();
        const Type_fr_trunk_X_fr_LF_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_upperleg : public TransformForce<Scalar, Type_fr_trunk_X_fr_RF_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_upperleg();
        const Type_fr_trunk_X_fr_RF_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_upperleg : public TransformForce<Scalar, Type_fr_trunk_X_fr_LH_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_upperleg();
        const Type_fr_trunk_X_fr_LH_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_upperleg : public TransformForce<Scalar, Type_fr_trunk_X_fr_RH_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_upperleg();
        const Type_fr_trunk_X_fr_RH_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_lowerleg : public TransformForce<Scalar, Type_fr_trunk_X_fr_LF_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_lowerleg();
        const Type_fr_trunk_X_fr_LF_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_lowerleg : public TransformForce<Scalar, Type_fr_trunk_X_fr_RF_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_lowerleg();
        const Type_fr_trunk_X_fr_RF_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_lowerleg : public TransformForce<Scalar, Type_fr_trunk_X_fr_LH_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_lowerleg();
        const Type_fr_trunk_X_fr_LH_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_lowerleg : public TransformForce<Scalar, Type_fr_trunk_X_fr_RH_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_lowerleg();
        const Type_fr_trunk_X_fr_RH_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_hipassembly_X_fr_trunk : public TransformForce<Scalar, Type_fr_LF_hipassembly_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_hipassembly_X_fr_trunk();
        const Type_fr_LF_hipassembly_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_hipassembly_X_fr_trunk : public TransformForce<Scalar, Type_fr_RF_hipassembly_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_hipassembly_X_fr_trunk();
        const Type_fr_RF_hipassembly_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_hipassembly_X_fr_trunk : public TransformForce<Scalar, Type_fr_LH_hipassembly_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_hipassembly_X_fr_trunk();
        const Type_fr_LH_hipassembly_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_hipassembly_X_fr_trunk : public TransformForce<Scalar, Type_fr_RH_hipassembly_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_hipassembly_X_fr_trunk();
        const Type_fr_RH_hipassembly_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_upperleg_X_fr_trunk : public TransformForce<Scalar, Type_fr_LF_upperleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_upperleg_X_fr_trunk();
        const Type_fr_LF_upperleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_upperleg_X_fr_trunk : public TransformForce<Scalar, Type_fr_RF_upperleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_upperleg_X_fr_trunk();
        const Type_fr_RF_upperleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_upperleg_X_fr_trunk : public TransformForce<Scalar, Type_fr_LH_upperleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_upperleg_X_fr_trunk();
        const Type_fr_LH_upperleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_upperleg_X_fr_trunk : public TransformForce<Scalar, Type_fr_RH_upperleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_upperleg_X_fr_trunk();
        const Type_fr_RH_upperleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_lowerleg_X_fr_trunk : public TransformForce<Scalar, Type_fr_LF_lowerleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_lowerleg_X_fr_trunk();
        const Type_fr_LF_lowerleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_lowerleg_X_fr_trunk : public TransformForce<Scalar, Type_fr_RF_lowerleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_lowerleg_X_fr_trunk();
        const Type_fr_RF_lowerleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_lowerleg_X_fr_trunk : public TransformForce<Scalar, Type_fr_LH_lowerleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_lowerleg_X_fr_trunk();
        const Type_fr_LH_lowerleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_lowerleg_X_fr_trunk : public TransformForce<Scalar, Type_fr_RH_lowerleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_lowerleg_X_fr_trunk();
        const Type_fr_RH_lowerleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_LF_hipassemblyCOM : public TransformForce<Scalar, Type_fr_trunk_X_LF_hipassemblyCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_LF_hipassemblyCOM();
        const Type_fr_trunk_X_LF_hipassemblyCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_hipassemblyCOM : public TransformForce<Scalar, Type_fr_trunk_X_RF_hipassemblyCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_RF_hipassemblyCOM();
        const Type_fr_trunk_X_RF_hipassemblyCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_LH_hipassemblyCOM : public TransformForce<Scalar, Type_fr_trunk_X_LH_hipassemblyCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_LH_hipassemblyCOM();
        const Type_fr_trunk_X_LH_hipassemblyCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_RH_hipassemblyCOM : public TransformForce<Scalar, Type_fr_trunk_X_RH_hipassemblyCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_RH_hipassemblyCOM();
        const Type_fr_trunk_X_RH_hipassemblyCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_LF_upperlegCOM : public TransformForce<Scalar, Type_fr_trunk_X_LF_upperlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_LF_upperlegCOM();
        const Type_fr_trunk_X_LF_upperlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_upperlegCOM : public TransformForce<Scalar, Type_fr_trunk_X_RF_upperlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_RF_upperlegCOM();
        const Type_fr_trunk_X_RF_upperlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_LH_upperlegCOM : public TransformForce<Scalar, Type_fr_trunk_X_LH_upperlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_LH_upperlegCOM();
        const Type_fr_trunk_X_LH_upperlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_RH_upperlegCOM : public TransformForce<Scalar, Type_fr_trunk_X_RH_upperlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_RH_upperlegCOM();
        const Type_fr_trunk_X_RH_upperlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_LF_lowerlegCOM : public TransformForce<Scalar, Type_fr_trunk_X_LF_lowerlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_LF_lowerlegCOM();
        const Type_fr_trunk_X_LF_lowerlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_lowerlegCOM : public TransformForce<Scalar, Type_fr_trunk_X_RF_lowerlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_RF_lowerlegCOM();
        const Type_fr_trunk_X_RF_lowerlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_LH_lowerlegCOM : public TransformForce<Scalar, Type_fr_trunk_X_LH_lowerlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_LH_lowerlegCOM();
        const Type_fr_trunk_X_LH_lowerlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_RH_lowerlegCOM : public TransformForce<Scalar, Type_fr_trunk_X_RH_lowerlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_RH_lowerlegCOM();
        const Type_fr_trunk_X_RH_lowerlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_foot_X_fr_LF_lowerleg : public TransformForce<Scalar, Type_fr_LF_foot_X_fr_LF_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_foot_X_fr_LF_lowerleg();
        const Type_fr_LF_foot_X_fr_LF_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_foot_X_fr_RF_lowerleg : public TransformForce<Scalar, Type_fr_RF_foot_X_fr_RF_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_foot_X_fr_RF_lowerleg();
        const Type_fr_RF_foot_X_fr_RF_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_foot_X_fr_LH_lowerleg : public TransformForce<Scalar, Type_fr_LH_foot_X_fr_LH_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_foot_X_fr_LH_lowerleg();
        const Type_fr_LH_foot_X_fr_LH_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_foot_X_fr_RH_lowerleg : public TransformForce<Scalar, Type_fr_RH_foot_X_fr_RH_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_foot_X_fr_RH_lowerleg();
        const Type_fr_RH_foot_X_fr_RH_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_foot : public TransformForce<Scalar, Type_fr_trunk_X_fr_LF_foot>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_foot();
        const Type_fr_trunk_X_fr_LF_foot& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_foot : public TransformForce<Scalar, Type_fr_trunk_X_fr_RF_foot>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_foot();
        const Type_fr_trunk_X_fr_RF_foot& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_foot : public TransformForce<Scalar, Type_fr_trunk_X_fr_LH_foot>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_foot();
        const Type_fr_trunk_X_fr_LH_foot& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_foot : public TransformForce<Scalar, Type_fr_trunk_X_fr_RH_foot>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_foot();
        const Type_fr_trunk_X_fr_RH_foot& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_foot_X_fr_trunk : public TransformForce<Scalar, Type_fr_LF_foot_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_foot_X_fr_trunk();
        const Type_fr_LF_foot_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_foot_X_fr_trunk : public TransformForce<Scalar, Type_fr_RF_foot_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_foot_X_fr_trunk();
        const Type_fr_RF_foot_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_foot_X_fr_trunk : public TransformForce<Scalar, Type_fr_LH_foot_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_foot_X_fr_trunk();
        const Type_fr_LH_foot_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_foot_X_fr_trunk : public TransformForce<Scalar, Type_fr_RH_foot_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_foot_X_fr_trunk();
        const Type_fr_RH_foot_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_HAA : public TransformForce<Scalar, Type_fr_trunk_X_fr_LF_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_HAA();
        const Type_fr_trunk_X_fr_LF_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_HAA : public TransformForce<Scalar, Type_fr_trunk_X_fr_RF_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_HAA();
        const Type_fr_trunk_X_fr_RF_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_HAA : public TransformForce<Scalar, Type_fr_trunk_X_fr_LH_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_HAA();
        const Type_fr_trunk_X_fr_LH_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_HAA : public TransformForce<Scalar, Type_fr_trunk_X_fr_RH_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_HAA();
        const Type_fr_trunk_X_fr_RH_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_HFE : public TransformForce<Scalar, Type_fr_trunk_X_fr_LF_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_HFE();
        const Type_fr_trunk_X_fr_LF_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_HFE : public TransformForce<Scalar, Type_fr_trunk_X_fr_RF_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_HFE();
        const Type_fr_trunk_X_fr_RF_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_HFE : public TransformForce<Scalar, Type_fr_trunk_X_fr_LH_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_HFE();
        const Type_fr_trunk_X_fr_LH_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_HFE : public TransformForce<Scalar, Type_fr_trunk_X_fr_RH_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_HFE();
        const Type_fr_trunk_X_fr_RH_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_KFE : public TransformForce<Scalar, Type_fr_trunk_X_fr_LF_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_KFE();
        const Type_fr_trunk_X_fr_LF_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_KFE : public TransformForce<Scalar, Type_fr_trunk_X_fr_RF_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_KFE();
        const Type_fr_trunk_X_fr_RF_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_KFE : public TransformForce<Scalar, Type_fr_trunk_X_fr_LH_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_KFE();
        const Type_fr_trunk_X_fr_LH_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_KFE : public TransformForce<Scalar, Type_fr_trunk_X_fr_RH_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_KFE();
        const Type_fr_trunk_X_fr_RH_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_upperleg_X_fr_LF_hipassembly : public TransformForce<Scalar, Type_fr_LF_upperleg_X_fr_LF_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_upperleg_X_fr_LF_hipassembly();
        const Type_fr_LF_upperleg_X_fr_LF_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_hipassembly_X_fr_LF_upperleg : public TransformForce<Scalar, Type_fr_LF_hipassembly_X_fr_LF_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_hipassembly_X_fr_LF_upperleg();
        const Type_fr_LF_hipassembly_X_fr_LF_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_lowerleg_X_fr_LF_upperleg : public TransformForce<Scalar, Type_fr_LF_lowerleg_X_fr_LF_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_lowerleg_X_fr_LF_upperleg();
        const Type_fr_LF_lowerleg_X_fr_LF_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_upperleg_X_fr_LF_lowerleg : public TransformForce<Scalar, Type_fr_LF_upperleg_X_fr_LF_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_upperleg_X_fr_LF_lowerleg();
        const Type_fr_LF_upperleg_X_fr_LF_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_upperleg_X_fr_RF_hipassembly : public TransformForce<Scalar, Type_fr_RF_upperleg_X_fr_RF_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_upperleg_X_fr_RF_hipassembly();
        const Type_fr_RF_upperleg_X_fr_RF_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_hipassembly_X_fr_RF_upperleg : public TransformForce<Scalar, Type_fr_RF_hipassembly_X_fr_RF_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_hipassembly_X_fr_RF_upperleg();
        const Type_fr_RF_hipassembly_X_fr_RF_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_lowerleg_X_fr_RF_upperleg : public TransformForce<Scalar, Type_fr_RF_lowerleg_X_fr_RF_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_lowerleg_X_fr_RF_upperleg();
        const Type_fr_RF_lowerleg_X_fr_RF_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_upperleg_X_fr_RF_lowerleg : public TransformForce<Scalar, Type_fr_RF_upperleg_X_fr_RF_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_upperleg_X_fr_RF_lowerleg();
        const Type_fr_RF_upperleg_X_fr_RF_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_upperleg_X_fr_LH_hipassembly : public TransformForce<Scalar, Type_fr_LH_upperleg_X_fr_LH_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_upperleg_X_fr_LH_hipassembly();
        const Type_fr_LH_upperleg_X_fr_LH_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_hipassembly_X_fr_LH_upperleg : public TransformForce<Scalar, Type_fr_LH_hipassembly_X_fr_LH_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_hipassembly_X_fr_LH_upperleg();
        const Type_fr_LH_hipassembly_X_fr_LH_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_lowerleg_X_fr_LH_upperleg : public TransformForce<Scalar, Type_fr_LH_lowerleg_X_fr_LH_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_lowerleg_X_fr_LH_upperleg();
        const Type_fr_LH_lowerleg_X_fr_LH_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_upperleg_X_fr_LH_lowerleg : public TransformForce<Scalar, Type_fr_LH_upperleg_X_fr_LH_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_upperleg_X_fr_LH_lowerleg();
        const Type_fr_LH_upperleg_X_fr_LH_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_upperleg_X_fr_RH_hipassembly : public TransformForce<Scalar, Type_fr_RH_upperleg_X_fr_RH_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_upperleg_X_fr_RH_hipassembly();
        const Type_fr_RH_upperleg_X_fr_RH_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_hipassembly_X_fr_RH_upperleg : public TransformForce<Scalar, Type_fr_RH_hipassembly_X_fr_RH_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_hipassembly_X_fr_RH_upperleg();
        const Type_fr_RH_hipassembly_X_fr_RH_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_lowerleg_X_fr_RH_upperleg : public TransformForce<Scalar, Type_fr_RH_lowerleg_X_fr_RH_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_lowerleg_X_fr_RH_upperleg();
        const Type_fr_RH_lowerleg_X_fr_RH_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_upperleg_X_fr_RH_lowerleg : public TransformForce<Scalar, Type_fr_RH_upperleg_X_fr_RH_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_upperleg_X_fr_RH_lowerleg();
        const Type_fr_RH_upperleg_X_fr_RH_lowerleg& update(const JState&);
    protected:
    };
    
public:
    ForceTransforms();
    void updateParameters();
    Type_fr_trunk_X_fr_LF_hipassembly fr_trunk_X_fr_LF_hipassembly;
    Type_fr_trunk_X_fr_RF_hipassembly fr_trunk_X_fr_RF_hipassembly;
    Type_fr_trunk_X_fr_LH_hipassembly fr_trunk_X_fr_LH_hipassembly;
    Type_fr_trunk_X_fr_RH_hipassembly fr_trunk_X_fr_RH_hipassembly;
    Type_fr_trunk_X_fr_LF_upperleg fr_trunk_X_fr_LF_upperleg;
    Type_fr_trunk_X_fr_RF_upperleg fr_trunk_X_fr_RF_upperleg;
    Type_fr_trunk_X_fr_LH_upperleg fr_trunk_X_fr_LH_upperleg;
    Type_fr_trunk_X_fr_RH_upperleg fr_trunk_X_fr_RH_upperleg;
    Type_fr_trunk_X_fr_LF_lowerleg fr_trunk_X_fr_LF_lowerleg;
    Type_fr_trunk_X_fr_RF_lowerleg fr_trunk_X_fr_RF_lowerleg;
    Type_fr_trunk_X_fr_LH_lowerleg fr_trunk_X_fr_LH_lowerleg;
    Type_fr_trunk_X_fr_RH_lowerleg fr_trunk_X_fr_RH_lowerleg;
    Type_fr_LF_hipassembly_X_fr_trunk fr_LF_hipassembly_X_fr_trunk;
    Type_fr_RF_hipassembly_X_fr_trunk fr_RF_hipassembly_X_fr_trunk;
    Type_fr_LH_hipassembly_X_fr_trunk fr_LH_hipassembly_X_fr_trunk;
    Type_fr_RH_hipassembly_X_fr_trunk fr_RH_hipassembly_X_fr_trunk;
    Type_fr_LF_upperleg_X_fr_trunk fr_LF_upperleg_X_fr_trunk;
    Type_fr_RF_upperleg_X_fr_trunk fr_RF_upperleg_X_fr_trunk;
    Type_fr_LH_upperleg_X_fr_trunk fr_LH_upperleg_X_fr_trunk;
    Type_fr_RH_upperleg_X_fr_trunk fr_RH_upperleg_X_fr_trunk;
    Type_fr_LF_lowerleg_X_fr_trunk fr_LF_lowerleg_X_fr_trunk;
    Type_fr_RF_lowerleg_X_fr_trunk fr_RF_lowerleg_X_fr_trunk;
    Type_fr_LH_lowerleg_X_fr_trunk fr_LH_lowerleg_X_fr_trunk;
    Type_fr_RH_lowerleg_X_fr_trunk fr_RH_lowerleg_X_fr_trunk;
    Type_fr_trunk_X_LF_hipassemblyCOM fr_trunk_X_LF_hipassemblyCOM;
    Type_fr_trunk_X_RF_hipassemblyCOM fr_trunk_X_RF_hipassemblyCOM;
    Type_fr_trunk_X_LH_hipassemblyCOM fr_trunk_X_LH_hipassemblyCOM;
    Type_fr_trunk_X_RH_hipassemblyCOM fr_trunk_X_RH_hipassemblyCOM;
    Type_fr_trunk_X_LF_upperlegCOM fr_trunk_X_LF_upperlegCOM;
    Type_fr_trunk_X_RF_upperlegCOM fr_trunk_X_RF_upperlegCOM;
    Type_fr_trunk_X_LH_upperlegCOM fr_trunk_X_LH_upperlegCOM;
    Type_fr_trunk_X_RH_upperlegCOM fr_trunk_X_RH_upperlegCOM;
    Type_fr_trunk_X_LF_lowerlegCOM fr_trunk_X_LF_lowerlegCOM;
    Type_fr_trunk_X_RF_lowerlegCOM fr_trunk_X_RF_lowerlegCOM;
    Type_fr_trunk_X_LH_lowerlegCOM fr_trunk_X_LH_lowerlegCOM;
    Type_fr_trunk_X_RH_lowerlegCOM fr_trunk_X_RH_lowerlegCOM;
    Type_fr_LF_foot_X_fr_LF_lowerleg fr_LF_foot_X_fr_LF_lowerleg;
    Type_fr_RF_foot_X_fr_RF_lowerleg fr_RF_foot_X_fr_RF_lowerleg;
    Type_fr_LH_foot_X_fr_LH_lowerleg fr_LH_foot_X_fr_LH_lowerleg;
    Type_fr_RH_foot_X_fr_RH_lowerleg fr_RH_foot_X_fr_RH_lowerleg;
    Type_fr_trunk_X_fr_LF_foot fr_trunk_X_fr_LF_foot;
    Type_fr_trunk_X_fr_RF_foot fr_trunk_X_fr_RF_foot;
    Type_fr_trunk_X_fr_LH_foot fr_trunk_X_fr_LH_foot;
    Type_fr_trunk_X_fr_RH_foot fr_trunk_X_fr_RH_foot;
    Type_fr_LF_foot_X_fr_trunk fr_LF_foot_X_fr_trunk;
    Type_fr_RF_foot_X_fr_trunk fr_RF_foot_X_fr_trunk;
    Type_fr_LH_foot_X_fr_trunk fr_LH_foot_X_fr_trunk;
    Type_fr_RH_foot_X_fr_trunk fr_RH_foot_X_fr_trunk;
    Type_fr_trunk_X_fr_LF_HAA fr_trunk_X_fr_LF_HAA;
    Type_fr_trunk_X_fr_RF_HAA fr_trunk_X_fr_RF_HAA;
    Type_fr_trunk_X_fr_LH_HAA fr_trunk_X_fr_LH_HAA;
    Type_fr_trunk_X_fr_RH_HAA fr_trunk_X_fr_RH_HAA;
    Type_fr_trunk_X_fr_LF_HFE fr_trunk_X_fr_LF_HFE;
    Type_fr_trunk_X_fr_RF_HFE fr_trunk_X_fr_RF_HFE;
    Type_fr_trunk_X_fr_LH_HFE fr_trunk_X_fr_LH_HFE;
    Type_fr_trunk_X_fr_RH_HFE fr_trunk_X_fr_RH_HFE;
    Type_fr_trunk_X_fr_LF_KFE fr_trunk_X_fr_LF_KFE;
    Type_fr_trunk_X_fr_RF_KFE fr_trunk_X_fr_RF_KFE;
    Type_fr_trunk_X_fr_LH_KFE fr_trunk_X_fr_LH_KFE;
    Type_fr_trunk_X_fr_RH_KFE fr_trunk_X_fr_RH_KFE;
    Type_fr_LF_upperleg_X_fr_LF_hipassembly fr_LF_upperleg_X_fr_LF_hipassembly;
    Type_fr_LF_hipassembly_X_fr_LF_upperleg fr_LF_hipassembly_X_fr_LF_upperleg;
    Type_fr_LF_lowerleg_X_fr_LF_upperleg fr_LF_lowerleg_X_fr_LF_upperleg;
    Type_fr_LF_upperleg_X_fr_LF_lowerleg fr_LF_upperleg_X_fr_LF_lowerleg;
    Type_fr_RF_upperleg_X_fr_RF_hipassembly fr_RF_upperleg_X_fr_RF_hipassembly;
    Type_fr_RF_hipassembly_X_fr_RF_upperleg fr_RF_hipassembly_X_fr_RF_upperleg;
    Type_fr_RF_lowerleg_X_fr_RF_upperleg fr_RF_lowerleg_X_fr_RF_upperleg;
    Type_fr_RF_upperleg_X_fr_RF_lowerleg fr_RF_upperleg_X_fr_RF_lowerleg;
    Type_fr_LH_upperleg_X_fr_LH_hipassembly fr_LH_upperleg_X_fr_LH_hipassembly;
    Type_fr_LH_hipassembly_X_fr_LH_upperleg fr_LH_hipassembly_X_fr_LH_upperleg;
    Type_fr_LH_lowerleg_X_fr_LH_upperleg fr_LH_lowerleg_X_fr_LH_upperleg;
    Type_fr_LH_upperleg_X_fr_LH_lowerleg fr_LH_upperleg_X_fr_LH_lowerleg;
    Type_fr_RH_upperleg_X_fr_RH_hipassembly fr_RH_upperleg_X_fr_RH_hipassembly;
    Type_fr_RH_hipassembly_X_fr_RH_upperleg fr_RH_hipassembly_X_fr_RH_upperleg;
    Type_fr_RH_lowerleg_X_fr_RH_upperleg fr_RH_lowerleg_X_fr_RH_upperleg;
    Type_fr_RH_upperleg_X_fr_RH_lowerleg fr_RH_upperleg_X_fr_RH_lowerleg;

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
    class Type_fr_trunk_X_fr_LF_hipassembly : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_LF_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_hipassembly();
        const Type_fr_trunk_X_fr_LF_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_hipassembly : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_RF_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_hipassembly();
        const Type_fr_trunk_X_fr_RF_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_hipassembly : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_LH_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_hipassembly();
        const Type_fr_trunk_X_fr_LH_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_hipassembly : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_RH_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_hipassembly();
        const Type_fr_trunk_X_fr_RH_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_upperleg : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_LF_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_upperleg();
        const Type_fr_trunk_X_fr_LF_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_upperleg : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_RF_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_upperleg();
        const Type_fr_trunk_X_fr_RF_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_upperleg : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_LH_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_upperleg();
        const Type_fr_trunk_X_fr_LH_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_upperleg : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_RH_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_upperleg();
        const Type_fr_trunk_X_fr_RH_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_lowerleg : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_LF_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_lowerleg();
        const Type_fr_trunk_X_fr_LF_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_lowerleg : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_RF_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_lowerleg();
        const Type_fr_trunk_X_fr_RF_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_lowerleg : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_LH_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_lowerleg();
        const Type_fr_trunk_X_fr_LH_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_lowerleg : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_RH_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_lowerleg();
        const Type_fr_trunk_X_fr_RH_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_hipassembly_X_fr_trunk : public TransformHomogeneous<Scalar, Type_fr_LF_hipassembly_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_hipassembly_X_fr_trunk();
        const Type_fr_LF_hipassembly_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_hipassembly_X_fr_trunk : public TransformHomogeneous<Scalar, Type_fr_RF_hipassembly_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_hipassembly_X_fr_trunk();
        const Type_fr_RF_hipassembly_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_hipassembly_X_fr_trunk : public TransformHomogeneous<Scalar, Type_fr_LH_hipassembly_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_hipassembly_X_fr_trunk();
        const Type_fr_LH_hipassembly_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_hipassembly_X_fr_trunk : public TransformHomogeneous<Scalar, Type_fr_RH_hipassembly_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_hipassembly_X_fr_trunk();
        const Type_fr_RH_hipassembly_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_upperleg_X_fr_trunk : public TransformHomogeneous<Scalar, Type_fr_LF_upperleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_upperleg_X_fr_trunk();
        const Type_fr_LF_upperleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_upperleg_X_fr_trunk : public TransformHomogeneous<Scalar, Type_fr_RF_upperleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_upperleg_X_fr_trunk();
        const Type_fr_RF_upperleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_upperleg_X_fr_trunk : public TransformHomogeneous<Scalar, Type_fr_LH_upperleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_upperleg_X_fr_trunk();
        const Type_fr_LH_upperleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_upperleg_X_fr_trunk : public TransformHomogeneous<Scalar, Type_fr_RH_upperleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_upperleg_X_fr_trunk();
        const Type_fr_RH_upperleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_lowerleg_X_fr_trunk : public TransformHomogeneous<Scalar, Type_fr_LF_lowerleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_lowerleg_X_fr_trunk();
        const Type_fr_LF_lowerleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_lowerleg_X_fr_trunk : public TransformHomogeneous<Scalar, Type_fr_RF_lowerleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_lowerleg_X_fr_trunk();
        const Type_fr_RF_lowerleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_lowerleg_X_fr_trunk : public TransformHomogeneous<Scalar, Type_fr_LH_lowerleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_lowerleg_X_fr_trunk();
        const Type_fr_LH_lowerleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_lowerleg_X_fr_trunk : public TransformHomogeneous<Scalar, Type_fr_RH_lowerleg_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_lowerleg_X_fr_trunk();
        const Type_fr_RH_lowerleg_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_LF_hipassemblyCOM : public TransformHomogeneous<Scalar, Type_fr_trunk_X_LF_hipassemblyCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_LF_hipassemblyCOM();
        const Type_fr_trunk_X_LF_hipassemblyCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_hipassemblyCOM : public TransformHomogeneous<Scalar, Type_fr_trunk_X_RF_hipassemblyCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_RF_hipassemblyCOM();
        const Type_fr_trunk_X_RF_hipassemblyCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_LH_hipassemblyCOM : public TransformHomogeneous<Scalar, Type_fr_trunk_X_LH_hipassemblyCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_LH_hipassemblyCOM();
        const Type_fr_trunk_X_LH_hipassemblyCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_RH_hipassemblyCOM : public TransformHomogeneous<Scalar, Type_fr_trunk_X_RH_hipassemblyCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_RH_hipassemblyCOM();
        const Type_fr_trunk_X_RH_hipassemblyCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_LF_upperlegCOM : public TransformHomogeneous<Scalar, Type_fr_trunk_X_LF_upperlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_LF_upperlegCOM();
        const Type_fr_trunk_X_LF_upperlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_upperlegCOM : public TransformHomogeneous<Scalar, Type_fr_trunk_X_RF_upperlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_RF_upperlegCOM();
        const Type_fr_trunk_X_RF_upperlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_LH_upperlegCOM : public TransformHomogeneous<Scalar, Type_fr_trunk_X_LH_upperlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_LH_upperlegCOM();
        const Type_fr_trunk_X_LH_upperlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_RH_upperlegCOM : public TransformHomogeneous<Scalar, Type_fr_trunk_X_RH_upperlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_RH_upperlegCOM();
        const Type_fr_trunk_X_RH_upperlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_LF_lowerlegCOM : public TransformHomogeneous<Scalar, Type_fr_trunk_X_LF_lowerlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_LF_lowerlegCOM();
        const Type_fr_trunk_X_LF_lowerlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_lowerlegCOM : public TransformHomogeneous<Scalar, Type_fr_trunk_X_RF_lowerlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_RF_lowerlegCOM();
        const Type_fr_trunk_X_RF_lowerlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_LH_lowerlegCOM : public TransformHomogeneous<Scalar, Type_fr_trunk_X_LH_lowerlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_LH_lowerlegCOM();
        const Type_fr_trunk_X_LH_lowerlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_RH_lowerlegCOM : public TransformHomogeneous<Scalar, Type_fr_trunk_X_RH_lowerlegCOM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_RH_lowerlegCOM();
        const Type_fr_trunk_X_RH_lowerlegCOM& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_foot_X_fr_LF_lowerleg : public TransformHomogeneous<Scalar, Type_fr_LF_foot_X_fr_LF_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_foot_X_fr_LF_lowerleg();
        const Type_fr_LF_foot_X_fr_LF_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_foot_X_fr_RF_lowerleg : public TransformHomogeneous<Scalar, Type_fr_RF_foot_X_fr_RF_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_foot_X_fr_RF_lowerleg();
        const Type_fr_RF_foot_X_fr_RF_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_foot_X_fr_LH_lowerleg : public TransformHomogeneous<Scalar, Type_fr_LH_foot_X_fr_LH_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_foot_X_fr_LH_lowerleg();
        const Type_fr_LH_foot_X_fr_LH_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_foot_X_fr_RH_lowerleg : public TransformHomogeneous<Scalar, Type_fr_RH_foot_X_fr_RH_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_foot_X_fr_RH_lowerleg();
        const Type_fr_RH_foot_X_fr_RH_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_foot : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_LF_foot>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_foot();
        const Type_fr_trunk_X_fr_LF_foot& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_foot : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_RF_foot>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_foot();
        const Type_fr_trunk_X_fr_RF_foot& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_foot : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_LH_foot>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_foot();
        const Type_fr_trunk_X_fr_LH_foot& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_foot : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_RH_foot>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_foot();
        const Type_fr_trunk_X_fr_RH_foot& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_foot_X_fr_trunk : public TransformHomogeneous<Scalar, Type_fr_LF_foot_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_foot_X_fr_trunk();
        const Type_fr_LF_foot_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_foot_X_fr_trunk : public TransformHomogeneous<Scalar, Type_fr_RF_foot_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_foot_X_fr_trunk();
        const Type_fr_RF_foot_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_foot_X_fr_trunk : public TransformHomogeneous<Scalar, Type_fr_LH_foot_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_foot_X_fr_trunk();
        const Type_fr_LH_foot_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_foot_X_fr_trunk : public TransformHomogeneous<Scalar, Type_fr_RH_foot_X_fr_trunk>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_foot_X_fr_trunk();
        const Type_fr_RH_foot_X_fr_trunk& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_HAA : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_LF_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_HAA();
        const Type_fr_trunk_X_fr_LF_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_HAA : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_RF_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_HAA();
        const Type_fr_trunk_X_fr_RF_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_HAA : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_LH_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_HAA();
        const Type_fr_trunk_X_fr_LH_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_HAA : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_RH_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_HAA();
        const Type_fr_trunk_X_fr_RH_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_HFE : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_LF_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_HFE();
        const Type_fr_trunk_X_fr_LF_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_HFE : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_RF_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_HFE();
        const Type_fr_trunk_X_fr_RF_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_HFE : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_LH_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_HFE();
        const Type_fr_trunk_X_fr_LH_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_HFE : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_RH_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_HFE();
        const Type_fr_trunk_X_fr_RH_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_KFE : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_LF_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LF_KFE();
        const Type_fr_trunk_X_fr_LF_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_KFE : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_RF_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RF_KFE();
        const Type_fr_trunk_X_fr_RF_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LH_KFE : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_LH_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_LH_KFE();
        const Type_fr_trunk_X_fr_LH_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RH_KFE : public TransformHomogeneous<Scalar, Type_fr_trunk_X_fr_RH_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_trunk_X_fr_RH_KFE();
        const Type_fr_trunk_X_fr_RH_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_upperleg_X_fr_LF_hipassembly : public TransformHomogeneous<Scalar, Type_fr_LF_upperleg_X_fr_LF_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_upperleg_X_fr_LF_hipassembly();
        const Type_fr_LF_upperleg_X_fr_LF_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_hipassembly_X_fr_LF_upperleg : public TransformHomogeneous<Scalar, Type_fr_LF_hipassembly_X_fr_LF_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_hipassembly_X_fr_LF_upperleg();
        const Type_fr_LF_hipassembly_X_fr_LF_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_lowerleg_X_fr_LF_upperleg : public TransformHomogeneous<Scalar, Type_fr_LF_lowerleg_X_fr_LF_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_lowerleg_X_fr_LF_upperleg();
        const Type_fr_LF_lowerleg_X_fr_LF_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_upperleg_X_fr_LF_lowerleg : public TransformHomogeneous<Scalar, Type_fr_LF_upperleg_X_fr_LF_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_upperleg_X_fr_LF_lowerleg();
        const Type_fr_LF_upperleg_X_fr_LF_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_upperleg_X_fr_RF_hipassembly : public TransformHomogeneous<Scalar, Type_fr_RF_upperleg_X_fr_RF_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_upperleg_X_fr_RF_hipassembly();
        const Type_fr_RF_upperleg_X_fr_RF_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_hipassembly_X_fr_RF_upperleg : public TransformHomogeneous<Scalar, Type_fr_RF_hipassembly_X_fr_RF_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_hipassembly_X_fr_RF_upperleg();
        const Type_fr_RF_hipassembly_X_fr_RF_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_lowerleg_X_fr_RF_upperleg : public TransformHomogeneous<Scalar, Type_fr_RF_lowerleg_X_fr_RF_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_lowerleg_X_fr_RF_upperleg();
        const Type_fr_RF_lowerleg_X_fr_RF_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_upperleg_X_fr_RF_lowerleg : public TransformHomogeneous<Scalar, Type_fr_RF_upperleg_X_fr_RF_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_upperleg_X_fr_RF_lowerleg();
        const Type_fr_RF_upperleg_X_fr_RF_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_upperleg_X_fr_LH_hipassembly : public TransformHomogeneous<Scalar, Type_fr_LH_upperleg_X_fr_LH_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_upperleg_X_fr_LH_hipassembly();
        const Type_fr_LH_upperleg_X_fr_LH_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_hipassembly_X_fr_LH_upperleg : public TransformHomogeneous<Scalar, Type_fr_LH_hipassembly_X_fr_LH_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_hipassembly_X_fr_LH_upperleg();
        const Type_fr_LH_hipassembly_X_fr_LH_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_lowerleg_X_fr_LH_upperleg : public TransformHomogeneous<Scalar, Type_fr_LH_lowerleg_X_fr_LH_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_lowerleg_X_fr_LH_upperleg();
        const Type_fr_LH_lowerleg_X_fr_LH_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_upperleg_X_fr_LH_lowerleg : public TransformHomogeneous<Scalar, Type_fr_LH_upperleg_X_fr_LH_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_upperleg_X_fr_LH_lowerleg();
        const Type_fr_LH_upperleg_X_fr_LH_lowerleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_upperleg_X_fr_RH_hipassembly : public TransformHomogeneous<Scalar, Type_fr_RH_upperleg_X_fr_RH_hipassembly>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_upperleg_X_fr_RH_hipassembly();
        const Type_fr_RH_upperleg_X_fr_RH_hipassembly& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_hipassembly_X_fr_RH_upperleg : public TransformHomogeneous<Scalar, Type_fr_RH_hipassembly_X_fr_RH_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_hipassembly_X_fr_RH_upperleg();
        const Type_fr_RH_hipassembly_X_fr_RH_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_lowerleg_X_fr_RH_upperleg : public TransformHomogeneous<Scalar, Type_fr_RH_lowerleg_X_fr_RH_upperleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_lowerleg_X_fr_RH_upperleg();
        const Type_fr_RH_lowerleg_X_fr_RH_upperleg& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_upperleg_X_fr_RH_lowerleg : public TransformHomogeneous<Scalar, Type_fr_RH_upperleg_X_fr_RH_lowerleg>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_upperleg_X_fr_RH_lowerleg();
        const Type_fr_RH_upperleg_X_fr_RH_lowerleg& update(const JState&);
    protected:
    };
    
public:
    HomogeneousTransforms();
    void updateParameters();
    Type_fr_trunk_X_fr_LF_hipassembly fr_trunk_X_fr_LF_hipassembly;
    Type_fr_trunk_X_fr_RF_hipassembly fr_trunk_X_fr_RF_hipassembly;
    Type_fr_trunk_X_fr_LH_hipassembly fr_trunk_X_fr_LH_hipassembly;
    Type_fr_trunk_X_fr_RH_hipassembly fr_trunk_X_fr_RH_hipassembly;
    Type_fr_trunk_X_fr_LF_upperleg fr_trunk_X_fr_LF_upperleg;
    Type_fr_trunk_X_fr_RF_upperleg fr_trunk_X_fr_RF_upperleg;
    Type_fr_trunk_X_fr_LH_upperleg fr_trunk_X_fr_LH_upperleg;
    Type_fr_trunk_X_fr_RH_upperleg fr_trunk_X_fr_RH_upperleg;
    Type_fr_trunk_X_fr_LF_lowerleg fr_trunk_X_fr_LF_lowerleg;
    Type_fr_trunk_X_fr_RF_lowerleg fr_trunk_X_fr_RF_lowerleg;
    Type_fr_trunk_X_fr_LH_lowerleg fr_trunk_X_fr_LH_lowerleg;
    Type_fr_trunk_X_fr_RH_lowerleg fr_trunk_X_fr_RH_lowerleg;
    Type_fr_LF_hipassembly_X_fr_trunk fr_LF_hipassembly_X_fr_trunk;
    Type_fr_RF_hipassembly_X_fr_trunk fr_RF_hipassembly_X_fr_trunk;
    Type_fr_LH_hipassembly_X_fr_trunk fr_LH_hipassembly_X_fr_trunk;
    Type_fr_RH_hipassembly_X_fr_trunk fr_RH_hipassembly_X_fr_trunk;
    Type_fr_LF_upperleg_X_fr_trunk fr_LF_upperleg_X_fr_trunk;
    Type_fr_RF_upperleg_X_fr_trunk fr_RF_upperleg_X_fr_trunk;
    Type_fr_LH_upperleg_X_fr_trunk fr_LH_upperleg_X_fr_trunk;
    Type_fr_RH_upperleg_X_fr_trunk fr_RH_upperleg_X_fr_trunk;
    Type_fr_LF_lowerleg_X_fr_trunk fr_LF_lowerleg_X_fr_trunk;
    Type_fr_RF_lowerleg_X_fr_trunk fr_RF_lowerleg_X_fr_trunk;
    Type_fr_LH_lowerleg_X_fr_trunk fr_LH_lowerleg_X_fr_trunk;
    Type_fr_RH_lowerleg_X_fr_trunk fr_RH_lowerleg_X_fr_trunk;
    Type_fr_trunk_X_LF_hipassemblyCOM fr_trunk_X_LF_hipassemblyCOM;
    Type_fr_trunk_X_RF_hipassemblyCOM fr_trunk_X_RF_hipassemblyCOM;
    Type_fr_trunk_X_LH_hipassemblyCOM fr_trunk_X_LH_hipassemblyCOM;
    Type_fr_trunk_X_RH_hipassemblyCOM fr_trunk_X_RH_hipassemblyCOM;
    Type_fr_trunk_X_LF_upperlegCOM fr_trunk_X_LF_upperlegCOM;
    Type_fr_trunk_X_RF_upperlegCOM fr_trunk_X_RF_upperlegCOM;
    Type_fr_trunk_X_LH_upperlegCOM fr_trunk_X_LH_upperlegCOM;
    Type_fr_trunk_X_RH_upperlegCOM fr_trunk_X_RH_upperlegCOM;
    Type_fr_trunk_X_LF_lowerlegCOM fr_trunk_X_LF_lowerlegCOM;
    Type_fr_trunk_X_RF_lowerlegCOM fr_trunk_X_RF_lowerlegCOM;
    Type_fr_trunk_X_LH_lowerlegCOM fr_trunk_X_LH_lowerlegCOM;
    Type_fr_trunk_X_RH_lowerlegCOM fr_trunk_X_RH_lowerlegCOM;
    Type_fr_LF_foot_X_fr_LF_lowerleg fr_LF_foot_X_fr_LF_lowerleg;
    Type_fr_RF_foot_X_fr_RF_lowerleg fr_RF_foot_X_fr_RF_lowerleg;
    Type_fr_LH_foot_X_fr_LH_lowerleg fr_LH_foot_X_fr_LH_lowerleg;
    Type_fr_RH_foot_X_fr_RH_lowerleg fr_RH_foot_X_fr_RH_lowerleg;
    Type_fr_trunk_X_fr_LF_foot fr_trunk_X_fr_LF_foot;
    Type_fr_trunk_X_fr_RF_foot fr_trunk_X_fr_RF_foot;
    Type_fr_trunk_X_fr_LH_foot fr_trunk_X_fr_LH_foot;
    Type_fr_trunk_X_fr_RH_foot fr_trunk_X_fr_RH_foot;
    Type_fr_LF_foot_X_fr_trunk fr_LF_foot_X_fr_trunk;
    Type_fr_RF_foot_X_fr_trunk fr_RF_foot_X_fr_trunk;
    Type_fr_LH_foot_X_fr_trunk fr_LH_foot_X_fr_trunk;
    Type_fr_RH_foot_X_fr_trunk fr_RH_foot_X_fr_trunk;
    Type_fr_trunk_X_fr_LF_HAA fr_trunk_X_fr_LF_HAA;
    Type_fr_trunk_X_fr_RF_HAA fr_trunk_X_fr_RF_HAA;
    Type_fr_trunk_X_fr_LH_HAA fr_trunk_X_fr_LH_HAA;
    Type_fr_trunk_X_fr_RH_HAA fr_trunk_X_fr_RH_HAA;
    Type_fr_trunk_X_fr_LF_HFE fr_trunk_X_fr_LF_HFE;
    Type_fr_trunk_X_fr_RF_HFE fr_trunk_X_fr_RF_HFE;
    Type_fr_trunk_X_fr_LH_HFE fr_trunk_X_fr_LH_HFE;
    Type_fr_trunk_X_fr_RH_HFE fr_trunk_X_fr_RH_HFE;
    Type_fr_trunk_X_fr_LF_KFE fr_trunk_X_fr_LF_KFE;
    Type_fr_trunk_X_fr_RF_KFE fr_trunk_X_fr_RF_KFE;
    Type_fr_trunk_X_fr_LH_KFE fr_trunk_X_fr_LH_KFE;
    Type_fr_trunk_X_fr_RH_KFE fr_trunk_X_fr_RH_KFE;
    Type_fr_LF_upperleg_X_fr_LF_hipassembly fr_LF_upperleg_X_fr_LF_hipassembly;
    Type_fr_LF_hipassembly_X_fr_LF_upperleg fr_LF_hipassembly_X_fr_LF_upperleg;
    Type_fr_LF_lowerleg_X_fr_LF_upperleg fr_LF_lowerleg_X_fr_LF_upperleg;
    Type_fr_LF_upperleg_X_fr_LF_lowerleg fr_LF_upperleg_X_fr_LF_lowerleg;
    Type_fr_RF_upperleg_X_fr_RF_hipassembly fr_RF_upperleg_X_fr_RF_hipassembly;
    Type_fr_RF_hipassembly_X_fr_RF_upperleg fr_RF_hipassembly_X_fr_RF_upperleg;
    Type_fr_RF_lowerleg_X_fr_RF_upperleg fr_RF_lowerleg_X_fr_RF_upperleg;
    Type_fr_RF_upperleg_X_fr_RF_lowerleg fr_RF_upperleg_X_fr_RF_lowerleg;
    Type_fr_LH_upperleg_X_fr_LH_hipassembly fr_LH_upperleg_X_fr_LH_hipassembly;
    Type_fr_LH_hipassembly_X_fr_LH_upperleg fr_LH_hipassembly_X_fr_LH_upperleg;
    Type_fr_LH_lowerleg_X_fr_LH_upperleg fr_LH_lowerleg_X_fr_LH_upperleg;
    Type_fr_LH_upperleg_X_fr_LH_lowerleg fr_LH_upperleg_X_fr_LH_lowerleg;
    Type_fr_RH_upperleg_X_fr_RH_hipassembly fr_RH_upperleg_X_fr_RH_hipassembly;
    Type_fr_RH_hipassembly_X_fr_RH_upperleg fr_RH_hipassembly_X_fr_RH_upperleg;
    Type_fr_RH_lowerleg_X_fr_RH_upperleg fr_RH_lowerleg_X_fr_RH_upperleg;
    Type_fr_RH_upperleg_X_fr_RH_lowerleg fr_RH_upperleg_X_fr_RH_lowerleg;

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
