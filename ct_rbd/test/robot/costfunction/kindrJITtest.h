/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-value"
#include <kindr/Core>
#pragma GCC diagnostic pop

/*!
 * A 3x3 function which constructs a a kindr EulerAngles object in between.
 * The aim of this function is to verify that JIT gets along with kindr Euler angle types
 * (which it should do naturally)
 *
 * @param x the input
 * @return output y = f(x)
 */
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> testFunction(const Eigen::Matrix<SCALAR, 3, 1>& x)
{
    kindr::EulerAnglesXyz<SCALAR> anglesIn(x);
    kindr::EulerAnglesXyz<SCALAR> anglesTemp;

    anglesTemp.setX(3 * anglesIn.x() + 2 * anglesIn.x() * anglesIn.x() - anglesIn.y() * anglesIn.z());
    anglesTemp.setY(anglesIn.z() + anglesIn.y() + 3);
    anglesTemp.setZ(anglesIn.z());

    kindr::EulerAnglesXyz<SCALAR> anglesOut(anglesTemp);

    return anglesOut.toImplementation();
}

/*!
 * The analytically, manually derived Jacobian of testFunction() used for verification
 *
 * @param x the input
 * @return output y = f(x)
 */
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 3> jacobianCheck(const Eigen::Matrix<SCALAR, 3, 1>& x)
{
    Eigen::Matrix<SCALAR, 3, 3> jac;

    jac << 3 + 4 * x(0), -x(2), -x(1), 0, 1, 1, 0, 0, 1;

    return jac;
}


/*!
 * Test for just-in-time compilation of the Jacobian and
 * normal Auto-diff Jacobian, with kindr in the loop
 */
TEST(KindrJitTest, EulerAnglesTest)
{
    using derivativesCppadJIT = ct::core::DerivativesCppadJIT<3, 3>;
    using derivativesCppad = ct::core::DerivativesCppad<3, 3>;

    try
    {
        // create a function handle (also works for class methods, lambdas, function pointers, ...)
        typename derivativesCppadJIT::FUN_TYPE_CG f = testFunction<derivativesCppadJIT::CG_SCALAR>;
        typename derivativesCppad::FUN_TYPE_AD f_ad = testFunction<derivativesCppad::AD_SCALAR>;

        // initialize the Auto-Diff Codegen Jacobian
        derivativesCppadJIT jacCG(f);
        derivativesCppad jacAd(f_ad);

        DerivativesCppadSettings settings;
        settings.createJacobian_ = true;

        // compile the Jacobian
        jacCG.compileJIT(settings, "eulerAnglesTestLib");

        // create an input vector
        Eigen::Matrix<double, 3, 1> angles;

        for (size_t i = 0; i < 10; i++)
        {
            // create a random input
            angles.setRandom();

            // verify agains the analytical Jacobian
            ASSERT_LT((jacCG.jacobian(angles) - jacobianCheck(angles)).array().abs().maxCoeff(), 1e-10);
            ASSERT_LT((jacAd.jacobian(angles) - jacobianCheck(angles)).array().abs().maxCoeff(), 1e-10);
        }
    } catch (std::exception& e)
    {
        std::cout << "Kindr Euler angle test failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}


//! test the kindr rotate method on euler angles
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> rotateTestFunction(const Eigen::Matrix<SCALAR, 3, 1>& x)
{
    kindr::EulerAnglesXyz<SCALAR> angles(x);

    // create a position and set to a trivial hard-coded value
    kindr::Position<SCALAR, 3> pos;
    pos.toImplementation()(0) = 1;
    pos.toImplementation()(1) = 1;
    pos.toImplementation()(2) = 1;

    return angles.rotate(pos).toImplementation();
}
TEST(KindrJitTest, EulerRotateTest)
{
    using derivativesCppadJIT = ct::core::DerivativesCppadJIT<3, 3>;
    try
    {
        // create a function handle (also works for class methods, lambdas, function pointers, ...)
        typename derivativesCppadJIT::FUN_TYPE_CG f = rotateTestFunction<derivativesCppadJIT::CG_SCALAR>;

        // initialize the Auto-Diff Codegen Jacobian
        derivativesCppadJIT jacCG(f);

        DerivativesCppadSettings settings;
        settings.createJacobian_ = true;

        // compile the Jacobian
        jacCG.compileJIT(settings, "EulerRotateTest");

    } catch (std::exception& e)
    {
        std::cout << "Kindr rotate test failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}


//! test the kindr RotationMatrix constructor
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> kindrRotationMatrixTestFunction(const Eigen::Matrix<SCALAR, 3, 1>& x)
{
    kindr::EulerAnglesXyz<SCALAR> angles(x);

    // create a position and set to a trivial hard-coded value
    Eigen::Matrix<SCALAR, 3, 1> pos;
    pos(0) = 1;
    pos(1) = 1;
    pos(2) = 1;

    Eigen::Matrix<SCALAR, 3, 3> someMatrix;
    someMatrix.setIdentity();
    someMatrix(0, 0) = ct::core::internal::CppADCodegenTrait::cos(x(0));
    someMatrix(1, 1) = ct::core::internal::CppADCodegenTrait::cos(x(0));
    someMatrix(0, 1) = -ct::core::internal::CppADCodegenTrait::sin(x(0));
    someMatrix(1, 0) = ct::core::internal::CppADCodegenTrait::sin(x(0));

    kindr::RotationMatrix<SCALAR> rotMat(someMatrix);

    return rotMat.toImplementation() * pos;
}
TEST(KindrJitTest, RotationMatrixTest)
{
    using derivativesCppadJIT = ct::core::DerivativesCppadJIT<3, 3>;

    try
    {
        // create a function handle (also works for class methods, lambdas, function pointers, ...)
        typename derivativesCppadJIT::FUN_TYPE_CG f = kindrRotationMatrixTestFunction<derivativesCppadJIT::CG_SCALAR>;

        // initialize the Auto-Diff Codegen Jacobian
        derivativesCppadJIT jacCG(f);

        DerivativesCppadSettings settings;
        settings.createJacobian_ = true;
        settings.createForwardZero_ = true;

        // compile the Jacobian
        jacCG.compileJIT(settings, "rotationMatrixTestLib");

    } catch (std::exception& e)
    {
        std::cout << "Kindr rotation matrix test failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}


//! test the behaviour of transcribing a kindr rotation matrix into Euler Angles
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> convertRotationMatrixToKindrEulerTestFunction(const Eigen::Matrix<SCALAR, 3, 1>& x)
{
    Eigen::Matrix<SCALAR, 3, 3> someMatrix;
    someMatrix.setIdentity();
    someMatrix(0, 0) = ct::core::internal::CppADCodegenTrait::cos(x(0));
    someMatrix(1, 1) = ct::core::internal::CppADCodegenTrait::cos(x(0));
    someMatrix(0, 1) = -ct::core::internal::CppADCodegenTrait::sin(x(0));
    someMatrix(1, 0) = ct::core::internal::CppADCodegenTrait::sin(x(0));

    kindr::RotationMatrix<SCALAR> rotMat(someMatrix);

    kindr::EulerAnglesXyz<SCALAR> euler(rotMat);  //<-- fails

    return euler.toImplementation();
}
TEST(KindrJitTest, DISABLED_KindrRotationMatrixToEulerAnglesXyzTest)
{
    using derivativesCppadJIT = ct::core::DerivativesCppadJIT<3, 3>;

    try
    {
        // create a function handle (also works for class methods, lambdas, function pointers, ...)
        typename derivativesCppadJIT::FUN_TYPE_CG f =
            convertRotationMatrixToKindrEulerTestFunction<derivativesCppadJIT::CG_SCALAR>;

        // initialize the Auto-Diff Codegen Jacobian
        derivativesCppadJIT jacCG(f);

        DerivativesCppadSettings settings;
        settings.createJacobian_ = true;
        settings.createForwardZero_ = true;

        // compile the Jacobian
        jacCG.compileJIT(settings, "rotationMatrixToEulerTestLib");

    } catch (std::exception& e)
    {
        std::cout << "KindrRotationMatrixToEulerXyzTest test failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}


//! test the behaviour of transcribing an Eigen rotation matrix into Eigen Euler Angles
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> convertRotationMatrixToEigenEulerTestFunction(const Eigen::Matrix<SCALAR, -1, 1>& x)
{
    Eigen::Matrix<SCALAR, 3, 3> someMatrix(3, 3);
    someMatrix.setIdentity();
    someMatrix(0, 0) = ct::core::internal::CppADCodegenTrait::cos(x(0));
    someMatrix(1, 1) = ct::core::internal::CppADCodegenTrait::cos(x(0));
    someMatrix(0, 1) = -ct::core::internal::CppADCodegenTrait::sin(x(0));
    someMatrix(1, 0) = ct::core::internal::CppADCodegenTrait::sin(x(0));

    Eigen::Matrix<SCALAR, 3, 1> eigenEulerAngles = someMatrix.eulerAngles(0, 1, 2);  // <-- fails

    return eigenEulerAngles;
}
TEST(KindrJitTest, DISABLED_EigenRotationMatrixToEigenEulerAnglesTest)
{
    using derivativesCppadJIT = ct::core::DerivativesCppadJIT<3, 3>;

    try
    {
        // create a function handle (also works for class methods, lambdas, function pointers, ...)
        typename derivativesCppadJIT::FUN_TYPE_CG f =
            convertRotationMatrixToEigenEulerTestFunction<derivativesCppadJIT::CG_SCALAR>;

        // initialize the Auto-Diff Codegen Jacobian
        derivativesCppadJIT jacCG(f);

        DerivativesCppadSettings settings;
        settings.createJacobian_ = true;
        settings.createForwardZero_ = true;

        // compile the Jacobian
        //        Eigen::VectorXd test (3); test << 0.0, 0.0, 0.0;
        //        jacCG.forwardZero(test); // <--fails
        jacCG.compileJIT(settings, "EigenRotationMatrixToEigenEulerAnglesTest");

    } catch (std::exception& e)
    {
        std::cout << "EigenRotationMatrixToEigenEulerAnglesTest test failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}

/* //TODO: bring back this test
#if EIGEN_VERSION_AT_LEAST(3, 2, 7)  // prior versions have a bug
//! test the behaviour of transcribing an Eigen rotation matrix into Eigen AngleAxis representation
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> convertRotationMatrixToAngleAxisTestFunction(const Eigen::Matrix<SCALAR, -1, 1>& x)
{
    Eigen::Matrix<SCALAR, 3, 3> someMatrix(3, 3);
    someMatrix.setIdentity();
    someMatrix(0, 0) = ct::core::internal::CppADCodegenTrait::cos(x(0));
    someMatrix(1, 1) = ct::core::internal::CppADCodegenTrait::cos(x(0));
    someMatrix(0, 1) = -ct::core::internal::CppADCodegenTrait::sin(x(0));
    someMatrix(1, 0) = ct::core::internal::CppADCodegenTrait::sin(x(0));

    Eigen::AngleAxis<SCALAR> angleAxis(someMatrix);

    return angleAxis.axis();
}

TEST(KindrJitTest, DISABLED_EigenRotationMatrixToAngleAxisTest)
{
    using derivativesCppadJIT = ct::core::DerivativesCppadJIT<3, 3>;

    try
    {
        // create a function handle (also works for class methods, lambdas, function pointers, ...)
        typename derivativesCppadJIT::FUN_TYPE_CG f =
            convertRotationMatrixToAngleAxisTestFunction<derivativesCppadJIT::CG_SCALAR>;

        // initialize the Auto-Diff Codegen Jacobian
        derivativesCppadJIT jacCG(f);

        DerivativesCppadSettings settings;
        settings.createJacobian_ = true;

        // compile the Jacobian
        jacCG.compileJIT(settings, "EigenRotationMatrixToAngleAxisTest");

    } catch (std::exception& e)
    {
        std::cout << "EigenRotationMatrixToAngleAxisTest test failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}
#endif
*/

//! test the Frobenius norm on an EigenMatrix
template <typename SCALAR>
Eigen::Matrix<SCALAR, -1, 1> frobeniusNormTestFunction(const Eigen::Matrix<SCALAR, -1, 1>& x)
{
    Eigen::Matrix<SCALAR, 3, 3> someMatrix(3, 3);
    someMatrix.setIdentity();
    someMatrix(0, 0) = ct::core::internal::CppADCodegenTrait::cos(x(0));
    someMatrix(1, 1) = ct::core::internal::CppADCodegenTrait::cos(x(0));
    someMatrix(0, 1) = -ct::core::internal::CppADCodegenTrait::sin(x(0));
    someMatrix(1, 0) = ct::core::internal::CppADCodegenTrait::sin(x(0));

    Eigen::Matrix<SCALAR, 1, 1> someData;
    someData(0, 0) = someMatrix.norm();  // per default Eigen returns the Frobenius when calling norm() on a matrix.
    return someData;
}
TEST(KindrJitTest, FrobeniusNormTest)
{
    using derivativesCppadJIT = ct::core::DerivativesCppadJIT<3, 1>;

    try
    {
        // create a function handle (also works for class methods, lambdas, function pointers, ...)
        typename derivativesCppadJIT::FUN_TYPE_CG f = frobeniusNormTestFunction<derivativesCppadJIT::CG_SCALAR>;

        // initialize the Auto-Diff Codegen Jacobian
        derivativesCppadJIT jacCG(f);

        DerivativesCppadSettings settings;
        settings.createJacobian_ = true;

        // compile the Jacobian
        jacCG.compileJIT(settings, "FrobeniusNormTest");

    } catch (std::exception& e)
    {
        std::cout << "FrobeniusNormTest test failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}


//! test the behaviour of transcribing a rotation matrix into a kindr RotationQuaternion
template <typename SCALAR>
Eigen::Matrix<SCALAR, 4, 1> convertRotationMatrixToKindrQuatTestFunction(const Eigen::Matrix<SCALAR, 3, 1>& x)
{
    Eigen::Matrix<SCALAR, 3, 3> someMatrix;
    someMatrix.setIdentity();
    someMatrix(0, 0) = ct::core::internal::CppADCodegenTrait::cos(x(0));
    someMatrix(1, 1) = ct::core::internal::CppADCodegenTrait::cos(x(0));
    someMatrix(0, 1) = -ct::core::internal::CppADCodegenTrait::sin(x(0));
    someMatrix(1, 0) = ct::core::internal::CppADCodegenTrait::sin(x(0));

    // this works
    kindr::RotationMatrix<SCALAR> rotMat(someMatrix);

    // this does not, although it should.
    kindr::RotationQuaternion<SCALAR> quat(rotMat);  // <-- todo this fails

    Eigen::Matrix<SCALAR, 4, 1> someData;
    someData << quat.w(), quat.x(), quat.y(), quat.z();

    return someData;
}
TEST(KindrJitTest, DISABLED_RotationMatrixToKindrRotationQuatTest)
{
    using derivativesCppadJIT = ct::core::DerivativesCppadJIT<3, 4>;

    try
    {
        // create a function handle (also works for class methods, lambdas, function pointers, ...)
        typename derivativesCppadJIT::FUN_TYPE_CG f =
            convertRotationMatrixToKindrQuatTestFunction<derivativesCppadJIT::CG_SCALAR>;

        // initialize the Auto-Diff Codegen Jacobian
        derivativesCppadJIT jacCG(f);

        DerivativesCppadSettings settings;
        settings.createJacobian_ = true;
        settings.createForwardZero_ = true;

        // compile the Jacobian
        jacCG.compileJIT(settings, "convertRotationMatrixToQuatTestFunction");

    } catch (std::exception& e)
    {
        std::cout << "convertRotationMatrixToQuatTestFunction test failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}


//! check conversion from kindr rotation matrix to an eigen quaternion
template <typename SCALAR>
Eigen::Matrix<SCALAR, 4, 1> convertKindrRotationMatrixToEigenQuatTestFunction(const Eigen::Matrix<SCALAR, 3, 1>& x)
{
    Eigen::Matrix<SCALAR, 3, 3> someMatrix;
    someMatrix.setIdentity();
    someMatrix(0, 0) = ct::core::internal::CppADCodegenTrait::cos(x(0));
    someMatrix(1, 1) = ct::core::internal::CppADCodegenTrait::cos(x(0));
    someMatrix(0, 1) = -ct::core::internal::CppADCodegenTrait::sin(x(0));
    someMatrix(1, 0) = ct::core::internal::CppADCodegenTrait::sin(x(0));

    kindr::RotationMatrix<SCALAR> rotMat(someMatrix);

    // this does not, although it should.
    Eigen::Quaternion<SCALAR> quat(someMatrix);  // <-- todo this fails

    Eigen::Matrix<SCALAR, 4, 1> someData;
    someData << quat.w(), quat.x(), quat.y(), quat.z();

    return someData;
}
TEST(KindrJitTest, DISABLED_KindrRotationMatrixToEigenRotationQuatTest)
{
    using derivativesCppadJIT = ct::core::DerivativesCppadJIT<3, 4>;

    try
    {
        // create a function handle (also works for class methods, lambdas, function pointers, ...)
        typename derivativesCppadJIT::FUN_TYPE_CG f =
            convertKindrRotationMatrixToEigenQuatTestFunction<derivativesCppadJIT::CG_SCALAR>;

        // initialize the Auto-Diff Codegen Jacobian
        derivativesCppadJIT jacCG(f);

        DerivativesCppadSettings settings;
        settings.createJacobian_ = true;
        settings.createForwardZero_ = true;

        // compile the Jacobian
        jacCG.compileJIT(settings, "RotationMatrixToEigenRotationQuatTest");

    } catch (std::exception& e)
    {
        std::cout << "RotationMatrixToEigenQuatTest test failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}


//! test the JIT compatibility of Eigen Transforms
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> eigenTransformTestFunction(const Eigen::Matrix<SCALAR, 3, 1>& x)
{
    Eigen::Transform<SCALAR, 3, Eigen::Affine> t;
    t = Eigen::Translation<SCALAR, 3>(x);
    t.rotate(Eigen::AngleAxis<SCALAR>(x(0), Eigen::Matrix<SCALAR, 3, 1>::UnitX()));
    t.rotate(Eigen::AngleAxis<SCALAR>(x(1), Eigen::Matrix<SCALAR, 3, 1>::UnitY()));
    t.rotate(Eigen::AngleAxis<SCALAR>(x(2), Eigen::Matrix<SCALAR, 3, 1>::UnitZ()));

    // return some random part of the eigen transform matrix
    return (t.matrix()).template topLeftCorner<3, 1>();
}
TEST(KindrJitTest, EigenTransformTest)
{
    using derivativesCppadJIT = ct::core::DerivativesCppadJIT<3, 3>;

    try
    {
        // create a function handle (also works for class methods, lambdas, function pointers, ...)
        typename derivativesCppadJIT::FUN_TYPE_CG f = eigenTransformTestFunction<derivativesCppadJIT::CG_SCALAR>;

        // initialize the Auto-Diff Codegen Jacobian
        derivativesCppadJIT jacCG(f);

        DerivativesCppadSettings settings;
        settings.createJacobian_ = true;

        // compile the Jacobian
        jacCG.compileJIT(settings, "eigenTransformTestLib");

    } catch (std::exception& e)
    {
        std::cout << "Eigen transform test failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}


//! test the JIT compatibility of Eigen quaternions
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> eigenQuaternionTestFunction(const Eigen::Matrix<SCALAR, 3, 1>& x)
{
    Eigen::Quaternion<SCALAR> q;
    q = Eigen::AngleAxis<SCALAR>(x(0), Eigen::Matrix<SCALAR, 3, 1>::UnitX()) *
        Eigen::AngleAxis<SCALAR>(x(1), Eigen::Matrix<SCALAR, 3, 1>::UnitY()) *
        Eigen::AngleAxis<SCALAR>(x(2), Eigen::Matrix<SCALAR, 3, 1>::UnitZ());

    // return some data
    return q.toRotationMatrix().template topRightCorner<3, 1>();
}
TEST(KindrJitTest, EigenQuaternionTest)
{
    using derivativesCppadJIT = ct::core::DerivativesCppadJIT<3, 3>;

    try
    {
        // create a function handle (also works for class methods, lambdas, function pointers, ...)
        typename derivativesCppadJIT::FUN_TYPE_CG f = eigenQuaternionTestFunction<derivativesCppadJIT::CG_SCALAR>;

        // initialize the Auto-Diff Codegen Jacobian
        derivativesCppadJIT jacCG(f);

        DerivativesCppadSettings settings;
        settings.createJacobian_ = true;

        // compile the Jacobian
        jacCG.compileJIT(settings, "EigenQuaternionTest");

    } catch (std::exception& e)
    {
        std::cout << "Eigen transform test failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}


//! check the conversion between an Eigen Quaternion and an kindr RotationQuaternion.
template <typename SCALAR>
Eigen::Matrix<SCALAR, 4, 1> eigenQuatToKindrRotationQuatFunction(const Eigen::Matrix<SCALAR, 3, 1>& x)
{
    Eigen::Quaternion<SCALAR> q;
    q = Eigen::AngleAxis<SCALAR>(x(0), Eigen::Matrix<SCALAR, 3, 1>::UnitX()) *
        Eigen::AngleAxis<SCALAR>(x(1), Eigen::Matrix<SCALAR, 3, 1>::UnitY()) *
        Eigen::AngleAxis<SCALAR>(x(2), Eigen::Matrix<SCALAR, 3, 1>::UnitZ());

    kindr::RotationQuaternion<SCALAR> kindr_quat(q);

    Eigen::Matrix<SCALAR, 4, 1> someData;
    someData << kindr_quat.w(), kindr_quat.x(), kindr_quat.y(), kindr_quat.z();

    return someData;
}
TEST(KindrJitTest, EigenQuatToKindrRotationQuatTest)
{
    using derivativesCppadJIT = ct::core::DerivativesCppadJIT<3, 4>;

    try
    {
        // create a function handle (also works for class methods, lambdas, function pointers, ...)
        typename derivativesCppadJIT::FUN_TYPE_CG f =
            eigenQuatToKindrRotationQuatFunction<derivativesCppadJIT::CG_SCALAR>;

        // initialize the Auto-Diff Codegen Jacobian
        derivativesCppadJIT jacCG(f);

        DerivativesCppadSettings settings;
        settings.createJacobian_ = true;

        // compile the Jacobian
        jacCG.compileJIT(settings, "EigenQuatToKindrRotationQuatTest");

    } catch (std::exception& e)
    {
        std::cout << "Eigen transform test failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}


//! test the JIT compatibility of kindr Quaternions
template <typename SCALAR>
Eigen::Matrix<SCALAR, 4, 1> kindrQuaternionTestFunction(const Eigen::Matrix<SCALAR, 3, 1>& x)
{
    kindr::Quaternion<SCALAR> q(x(0), x(1), x(2), x(2));

    Eigen::Matrix<SCALAR, 4, 1> res;
    res << q.w(), q.x(), q.y(), q.z();

    return res;
}
TEST(KindrJitTest, kindrQuaternionTest)
{
    using derivativesCppadJIT = ct::core::DerivativesCppadJIT<3, 4>;

    try
    {
        // create a function handle (also works for class methods, lambdas, function pointers, ...)
        typename derivativesCppadJIT::FUN_TYPE_CG f = kindrQuaternionTestFunction<derivativesCppadJIT::CG_SCALAR>;

        // initialize the Auto-Diff Codegen Jacobian
        derivativesCppadJIT jacCG(f);

        DerivativesCppadSettings settings;
        settings.createJacobian_ = true;

        // compile the Jacobian
        jacCG.compileJIT(settings, "kindrQuaternionTestTestLib");

    } catch (std::exception& e)
    {
        std::cout << "Eigen transform test failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}


//! test the JIT compatibility of kindr UnitQuaternions
template <typename SCALAR>
Eigen::Matrix<SCALAR, 4, 1> kindrUnitQuaternionTestFunction(const Eigen::Matrix<SCALAR, 3, 1>& x)
{
    Eigen::Quaternion<SCALAR> q_init;
    q_init = Eigen::AngleAxis<SCALAR>(x(0), Eigen::Matrix<SCALAR, 3, 1>::UnitX()) *
             Eigen::AngleAxis<SCALAR>(x(1), Eigen::Matrix<SCALAR, 3, 1>::UnitY()) *
             Eigen::AngleAxis<SCALAR>(x(2), Eigen::Matrix<SCALAR, 3, 1>::UnitZ());
    q_init = Eigen::Quaternion<SCALAR>((SCALAR)1.0, (SCALAR)0, (SCALAR)0, (SCALAR)0);

    kindr::UnitQuaternion<SCALAR> q(q_init);

    Eigen::Matrix<SCALAR, 4, 1> res;
    res << q.w(), q.x(), q.y(), q.z();

    return res;
}
TEST(KindrJitTest, kindrUnitQuaternionTest)
{
    using derivativesCppadJIT = ct::core::DerivativesCppadJIT<3, 4>;

    try
    {
        // create a function handle (also works for class methods, lambdas, function pointers, ...)
        typename derivativesCppadJIT::FUN_TYPE_CG f = kindrUnitQuaternionTestFunction<derivativesCppadJIT::CG_SCALAR>;

        // initialize the Auto-Diff Codegen Jacobian
        derivativesCppadJIT jacCG(f);

        DerivativesCppadSettings settings;
        settings.createJacobian_ = true;

        // compile the Jacobian
        jacCG.compileJIT(settings, "kindrUnitQuaternionTestLib");

    } catch (std::exception& e)
    {
        std::cout << "Eigen transform test failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}


/*!
 * Test JIT compatiblility of the kindr "RotationQuaternion" class
 */
template <typename SCALAR>
Eigen::Matrix<SCALAR, 4, 1> kindrRotationQuaternionTestFunction(const Eigen::Matrix<SCALAR, 3, 1>& x)
{
    kindr::RotationQuaternion<SCALAR> q(x(0), x(1), x(2), x(2));

    Eigen::Matrix<SCALAR, 4, 1> res;
    res << q.w(), q.x(), q.y(), q.z();

    return res;
}
TEST(KindrJitTest, kindrRotationQuaternionTest)
{
    using derivativesCppadJIT = ct::core::DerivativesCppadJIT<3, 4>;

    try
    {
        // create a function handle (also works for class methods, lambdas, function pointers, ...)
        typename derivativesCppadJIT::FUN_TYPE_CG f =
            kindrRotationQuaternionTestFunction<derivativesCppadJIT::CG_SCALAR>;

        // initialize the Auto-Diff Codegen Jacobian
        derivativesCppadJIT jacCG(f);

        DerivativesCppadSettings settings;
        settings.createJacobian_ = true;

        // compile the Jacobian
        jacCG.compileJIT(settings, "kindrRotationQuaternionTest");

    } catch (std::exception& e)
    {
        std::cout << "Eigen transform test failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}
