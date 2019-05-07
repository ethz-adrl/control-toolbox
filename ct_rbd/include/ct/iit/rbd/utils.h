/* CPYHDR { */
/*
 * This file is part of the 'iit-rbd' library.
 * Copyright © 2015 2016, Marco Frigerio (marco.frigerio@iit.it)
 *
 * See the LICENSE file for more information.
 */
/* } CPYHDR */
#ifndef IIT_RBD_UTILS_H_
#define IIT_RBD_UTILS_H_

#include <cmath>
#include <ctime>
#include <iostream>

#include "rbd.h"
#include "types.h"

namespace iit {
namespace rbd {

class Utils {
    public:
        struct InertiaMoments {
            double ixx; double iyy; double izz; double ixy; double ixz; double iyz;
        };

        /**
         * Coefficient-wise, unary operator that evaluates to zero for each element
         * of the matrix expression whose absolute value is below a threshold.
         * Useful for printing matrices, approximating with 0 small coefficients.
         */
        template <typename Scalar>
        struct CwiseAlmostZeroOp {
            CwiseAlmostZeroOp(const Scalar& threshold) : thresh(threshold) {}
            const Scalar operator()(const Scalar& x) const {
                return std::abs(x) < thresh ? 0 : x;
            }
            private:
                Scalar thresh;
        };
public:
    template <typename Scalar>
    static Mat33<Scalar> buildInertiaTensor(
            Scalar Ixx, Scalar Iyy, Scalar Izz,
            Scalar Ixy, Scalar Ixz, Scalar Iyz)
    {
        Mat33<Scalar> I;
        I <<  Ixx, -Ixy, -Ixz,
             -Ixy,  Iyy, -Iyz,
             -Ixz, -Iyz,  Izz;
        return I;
    }

    static Matrix33d buildCrossProductMatrix(const Vector3d& in) {
        Matrix33d out;
        out <<  0   , -in(2),  in(1),
               in(2),   0   , -in(0),
              -in(1),  in(0),   0;
        return out;
    }

    // Put implementation here in header even if it is not inline, since
    //  this is a template function
    /**
     * Suppose B is rotated respect to A by rx, than ry, than rz.
     * Fills the last parameter with the 3x3 rotation matrix A --> B ('B_X_A')
     */
    template <typename Derived>
    static void fillAsRotationMatrix(double rx, double ry, double rz,
            const MatrixBase<Derived>& mx)
    {
        eigen_assert(mx.rows() == 3  &&  mx.cols() == 3);

        static double cx;
        static double sx;
        static double cy;
        static double sy;
        static double cz;
        static double sz;
        cx = std::cos(rx);
        sx = std::sin(rx);
        cy = std::cos(ry);
        sy = std::sin(ry);
        cz = std::cos(rz);
        sz = std::sin(rz);

        const_cast<MatrixBase<Derived>&>(mx)
                   << cy*cz, cx*sz+sx*sy*cz, sx*sz-cx*sy*cz,
                     -cy*sz, cx*cz-sx*sy*sz, cx*sy*sz+sx*cz,
                      sy,   -sx*cy,         cx*cy;
    }

    /**
     * Suppose B is rotated respect to A by rx, than ry, than rz.
     * \return the 3x3 rotation matrix A --> B ('B_X_A')
     */
    static Matrix33d buildRotationMatrix(double rx, double ry, double rz) {
        double cx = std::cos(rx);
        double sx = std::sin(rx);
        double cy = std::cos(ry);
        double sy = std::sin(ry);
        double cz = std::cos(rz);
        double sz = std::sin(rz);
        Matrix33d ret;
        ret << cy*cz, cx*sz+sx*sy*cz, sx*sz-cx*sy*cz,
                -cy*sz, cx*cz-sx*sy*sz, cx*sy*sz+sx*cz,
                 sy,   -sx*cy,         cx*cy;
        return ret;
    }

#define block31 template block<3,1>
#define block33 template block<3,3>

    template <typename Scalar>
    static void fillAsMotionCrossProductMx(const Vec6<Scalar>& v, Mat66<Scalar>& mx) {
        fillAsCrossProductMatrix(v.block31(0,0), mx.block33(0,0));
        mx.block33(0,3).setZero();
        fillAsCrossProductMatrix(v.block31(3,0), mx.block33(3,0));
        mx.block33(3,3) = mx.block33(0,0);
    }
    template <typename Scalar>
    static void fillAsForceCrossProductMx(const Vec6<Scalar>& v, Mat66<Scalar>& mx) {
        fillAsCrossProductMatrix(v.block31(0,0), mx.block33(0,0));
        fillAsCrossProductMatrix(v.block31(3,0), mx.block33(0,3));
        mx.block33(3,0).setZero();
        mx.block33(3,3) = mx.block33(0,0);
    }

    template <typename D1, typename D2>
    static void fillAsCrossProductMatrix(const MatrixBase<D2>& in, const MatrixBase<D1>& mx)
    {
        eigen_assert(mx.rows() == 3  &&  mx.cols() == 3);
        typedef typename D2::Scalar Scalar;

        const_cast< MatrixBase<D1>& >(mx)
            <<  static_cast<Scalar>(0.0)   , -in(2),  in(1),
               in(2),   static_cast<Scalar>(0.0)   , -in(0),
              -in(1),  in(0),   static_cast<Scalar>(0.0);
    }

    template <typename Derived>
    static const MatrixBlock<const Derived,3,1> positionVector(const MatrixBase<Derived>& homT) {
        eigen_assert(homT.rows() == 4  &&  homT.cols() == 4); // weak way to check if it is a homogeneous transform
        return homT.block31(0,3);
    }
    template <typename Derived>
    static const MatrixBlock<const Derived,3,3> rotationMx(const MatrixBase<Derived>& homT) {
        eigen_assert(homT.rows() == 4  &&  homT.cols() == 4); // weak way to check if it is a homogeneous transform
        return homT.block33(0,0);
    }
    template <typename Derived>
    static const MatrixBlock<const Derived,3,1> zAxis(const MatrixBase<Derived>& homT) {
        eigen_assert(homT.rows() == 4  &&  homT.cols() == 4); // weak way to check if it is a homogeneous transform
        return homT.block31(0,2);
    }

#undef block31
#undef block33

    /**
     * Applies a roto-translation to the given 3D vector.
     * This function is provided for convenience, since a direct matrix product
     * between a 4x4 matrix and a 3x1 vector is obviously not possible. It should
     * also be slightly more efficient than taking the homogeneous representation
     * of the 3D vector (4x1) and then compute the matrix product.
     *
     * \param homT a 4x4 homogeneous coordinate transform encoding the rotation
     *        and the translation
     * \param vect3d the 3x1 vector to be transformed
     * \return the 3D vector that results from the rotation plus translation
     */
    template <typename Derived, typename Other>
    static Vector3d transform(
            const MatrixBase<Derived>& homT,
            const MatrixBase<Other>& vect3d)
    {
        eigen_assert(homT.rows() == 4  &&  homT.cols() == 4); // weak way to check if it is a homogeneous transform
        eigen_assert(vect3d.rows() == 3  &&  vect3d.cols() == 1); // check if it is a 3d column
        return rotationMx(homT) * vect3d + positionVector(homT);
    }

    static void randomVec(Vector6D& vec)
    {
        std::srand(std::time(NULL));
        for(int i=0; i<6; i++) {
            vec(i) = ((double)std::rand()) / RAND_MAX;
        }
    }

    static void randomGravity(VelocityVector& g)
    {
        angularPart(g).setZero();
        std::srand(std::time(NULL));
        g(LX) = ((double)std::rand()) / RAND_MAX;
        g(LY) = ((double)std::rand()) / RAND_MAX;
        g(LZ) = ((double)std::rand()) / RAND_MAX;
        linearPart(g) = (linearPart(g) / linearPart(g).norm()) * iit::rbd::g;
    }

    template <class RT>
    static void cmdlineargs_spatialv(int argc, char** argv,
            iit::rbd::Vector6D& vec)
    {
        if(argc < 6) {
            std::cerr << "Not enough arguments" << std::endl;
            exit(-1);
        }
        for(int i=0; i<6; i++) {
            vec(i) = std::atof(argv[i]);
        }
    }

};


}
}

#endif /* IIT_RBD_UTILS_H_ */
