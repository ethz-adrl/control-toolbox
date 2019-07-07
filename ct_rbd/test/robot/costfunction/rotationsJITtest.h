/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once


/*!
 * A 3x3 function which constructs a a EulerAngles object in between.
 * The aim of this function is to verify that JIT gets along with Euler angle types
 * (which it should do naturally)
 *
 * @param x the input
 * @return output y = f(x)
 */
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> testFunction(const Eigen::Matrix<SCALAR, 3, 1>& x)
{
    ct::rbd::tpl::EulerAnglesXYZ<SCALAR> anglesIn(x);
    ct::rbd::tpl::EulerAnglesXYZ<SCALAR> anglesTemp;

    anglesTemp.alpha() = 3 * anglesIn.x() + 2 * anglesIn.x() * anglesIn.x() - anglesIn.y() * anglesIn.z();
    anglesTemp.beta() = anglesIn.z() + anglesIn.y() + 3;
    anglesTemp.gamma() = anglesIn.z();

    ct::rbd::tpl::EulerAnglesXYZ<SCALAR> anglesOut(anglesTemp);

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
 * normal Auto-diff Jacobian
 */
TEST(RotationsJitTest, EulerAnglesTest)
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
        std::cout << "Euler angle test failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}


//! test the rotate method on euler angles
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> rotateTestFunction(const Eigen::Matrix<SCALAR, 3, 1>& x)
{
    ct::rbd::tpl::EulerAnglesXYZ<SCALAR> angles(x);

    // create a position and set to a trivial hard-coded value
    Eigen::Matrix<SCALAR, 3, 1> pos;
    pos(0) = 1;
    pos(1) = 1;
    pos(2) = 1;

    return angles.toRotationMatrix() * pos;
}
TEST(RotationsJitTest, EulerRotateTest)
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
        std::cout << "rotate test failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}


//! test the behaviour of transcribing a rotation matrix into Euler Angles
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> convertRotationMatrixToEulerTestFunction(const Eigen::Matrix<SCALAR, 3, 1>& x)
{
    Eigen::Matrix<SCALAR, 3, 3> rotMat;
    rotMat.setIdentity();
    rotMat(0, 0) = ct::core::internal::CppADCodegenTrait::cos(x(0));
    rotMat(1, 1) = ct::core::internal::CppADCodegenTrait::cos(x(0));
    rotMat(0, 1) = -ct::core::internal::CppADCodegenTrait::sin(x(0));
    rotMat(1, 0) = ct::core::internal::CppADCodegenTrait::sin(x(0));

    ct::rbd::tpl::EulerAnglesXYZ<SCALAR> euler(rotMat);  //<-- fails ?

    return euler.toImplementation();
}
TEST(RotationsJitTest, RotationMatrixToEulerAnglesXyzTest)
{
    using derivativesCppadJIT = ct::core::DerivativesCppadJIT<3, 3>;

    try
    {
        // create a function handle (also works for class methods, lambdas, function pointers, ...)
        typename derivativesCppadJIT::FUN_TYPE_CG f =
            convertRotationMatrixToEulerTestFunction<derivativesCppadJIT::CG_SCALAR>;

        // initialize the Auto-Diff Codegen Jacobian
        derivativesCppadJIT jacCG(f);

        DerivativesCppadSettings settings;
        settings.createJacobian_ = true;
        settings.createForwardZero_ = true;

        // compile the Jacobian
        jacCG.compileJIT(settings, "rotationMatrixToEulerTestLib");

    } catch (std::exception& e)
    {
        std::cout << "RotationMatrixToEulerXyzTest test failed: " << e.what() << std::endl;
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
TEST(RotationsJitTest, DISABLED_EigenRotationMatrixToEigenEulerAnglesTest)
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

TEST(RotationsJitTest, DISABLED_EigenRotationMatrixToAngleAxisTest)
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
TEST(RotationsJitTest, FrobeniusNormTest)
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


//! check conversion from rotation matrix to an eigen quaternion
template <typename SCALAR>
Eigen::Matrix<SCALAR, 4, 1> convertEigenRotationMatrixToEigenQuatTestFunction(const Eigen::Matrix<SCALAR, 3, 1>& x)
{
    Eigen::Matrix<SCALAR, 3, 3> someMatrix;
    someMatrix.setIdentity();
    someMatrix(0, 0) = ct::core::internal::CppADCodegenTrait::cos(x(0));
    someMatrix(1, 1) = ct::core::internal::CppADCodegenTrait::cos(x(0));
    someMatrix(0, 1) = -ct::core::internal::CppADCodegenTrait::sin(x(0));
    someMatrix(1, 0) = ct::core::internal::CppADCodegenTrait::sin(x(0));


    // this does not, although it should.
    Eigen::Quaternion<SCALAR> quat(someMatrix);  // <-- todo this fails

    Eigen::Matrix<SCALAR, 4, 1> someData;
    someData << quat.w(), quat.x(), quat.y(), quat.z();

    return someData;
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

TEST(RotationsJitTest, EigenTransformTest)
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

TEST(RotationsJitTest, EigenQuaternionTest)
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


//! test the JIT compatibility of eigen Quaternions
template <typename SCALAR>
Eigen::Matrix<SCALAR, 4, 1> eigenQuaternionTestFunction2(const Eigen::Matrix<SCALAR, 3, 1>& x)
{
    Eigen::Quaternion<SCALAR> q(x(0), x(1), x(2), x(2));

    Eigen::Matrix<SCALAR, 4, 1> res;
    res << q.w(), q.x(), q.y(), q.z();

    return res;
}

TEST(RotationsJitTest, eigenQuaternionTest2)
{
    using derivativesCppadJIT = ct::core::DerivativesCppadJIT<3, 4>;

    try
    {
        // create a function handle (also works for class methods, lambdas, function pointers, ...)
        typename derivativesCppadJIT::FUN_TYPE_CG f = eigenQuaternionTestFunction2<derivativesCppadJIT::CG_SCALAR>;

        // initialize the Auto-Diff Codegen Jacobian
        derivativesCppadJIT jacCG(f);

        DerivativesCppadSettings settings;
        settings.createJacobian_ = true;

        // compile the Jacobian
        jacCG.compileJIT(settings, "eigenQuaternionTest2TestLib");

    } catch (std::exception& e)
    {
        std::cout << "Eigen transform test failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}
