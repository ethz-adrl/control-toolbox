/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#ifndef CT_COSTFUNCTION_EXAMPLE_KINEMATICS_H_
#define CT_COSTFUNCTION_EXAMPLE_KINEMATICS_H_

#include <stdio.h>
#include <math.h>

namespace ct {
namespace optcon {
namespace example {

template <typename SCALAR>
class AD_Type_fr_link0_X_ee
{
public:
    enum JointIdentifiers
    {
        JA = 0,
        JB,
        JC,
        JD,
        JE,
        JF
    };

    AD_Type_fr_link0_X_ee() {}
    Eigen::Matrix<SCALAR, 4, 4> getHomogeneousTransform(const Eigen::Matrix<SCALAR, 6, 1>& q)
    {
        Eigen::Matrix<SCALAR, 4, 4> HomogeneousTransform;
        HomogeneousTransform.setZero();
        HomogeneousTransform(3, 3) = 1.0;


        // the following is copied from the code generator output transforms.cpp
        // to make your own example, replace all 'double' by 'SCALAR' and sin() and cos()
        // by CppAD::sin() / CppAD::cos()

        SCALAR sin__q_jB__ = CppAD::sin(q(JB));
        SCALAR sin__q_jC__ = CppAD::sin(q(JC));
        SCALAR sin__q_jE__ = CppAD::sin(q(JE));
        SCALAR sin__q_jA__ = CppAD::sin(q(JA));
        SCALAR sin__q_jD__ = CppAD::sin(q(JD));
        SCALAR sin__q_jF__ = CppAD::sin(q(JF));
        SCALAR cos__q_jA__ = CppAD::cos(q(JA));
        SCALAR cos__q_jB__ = CppAD::cos(q(JB));
        SCALAR cos__q_jC__ = CppAD::cos(q(JC));
        SCALAR cos__q_jD__ = CppAD::cos(q(JD));
        SCALAR cos__q_jE__ = CppAD::cos(q(JE));
        SCALAR cos__q_jF__ = CppAD::cos(q(JF));

        HomogeneousTransform(0, 0) = ((((((((((((0.6665 * cos__q_jA__) * cos__q_jB__) * cos__q_jC__) -
                                                 (((0.6665 * cos__q_jA__) * sin__q_jB__) * sin__q_jC__)) *
                                                sin__q_jE__) +
                                               ((((0.6665 * sin__q_jA__) * sin__q_jD__) +
                                                    (((((0.6665 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) +
                                                         (((0.6665 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                        cos__q_jD__)) *
                                                   cos__q_jE__)) +
                                              (((((0.4316 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) +
                                                   (((0.4316 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                  sin__q_jD__)) -
                                             ((0.4316 * sin__q_jA__) * cos__q_jD__)) *
                                            sin__q_jF__) +
                                           (((((((((0.4316 * cos__q_jA__) * sin__q_jB__) * sin__q_jC__) -
                                                    (((0.4316 * cos__q_jA__) * cos__q_jB__) * cos__q_jC__)) *
                                                   sin__q_jE__) +
                                                  (((((((-0.4316 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                         (((0.4316 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                        cos__q_jD__) -
                                                       ((0.4316 * sin__q_jA__) * sin__q_jD__)) *
                                                      cos__q_jE__)) +
                                                 (((((0.6665 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) +
                                                      (((0.6665 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                     sin__q_jD__)) -
                                                ((0.6665 * sin__q_jA__) * cos__q_jD__)) *
                                               cos__q_jF__)) +
                                          (((((((-0.6077 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                 (((0.6077 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                cos__q_jD__) -
                                               ((0.6077 * sin__q_jA__) * sin__q_jD__)) *
                                              sin__q_jE__)) +
                                      (((((0.6077 * cos__q_jA__) * cos__q_jB__) * cos__q_jC__) -
                                           (((0.6077 * cos__q_jA__) * sin__q_jB__) * sin__q_jC__)) *
                                          cos__q_jE__));
        HomogeneousTransform(0, 1) = ((((((((((((0.0316 * cos__q_jA__) * sin__q_jB__) * sin__q_jC__) -
                                                 (((0.0316 * cos__q_jA__) * cos__q_jB__) * cos__q_jC__)) *
                                                sin__q_jE__) +
                                               (((((((-0.0316 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                      (((0.0316 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                     cos__q_jD__) -
                                                    ((0.0316 * sin__q_jA__) * sin__q_jD__)) *
                                                   cos__q_jE__)) +
                                              (((((-0.7981 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                   (((0.7981 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                  sin__q_jD__)) +
                                             ((0.7981 * sin__q_jA__) * cos__q_jD__)) *
                                            sin__q_jF__) +
                                           (((((((((0.7981 * cos__q_jA__) * cos__q_jB__) * cos__q_jC__) -
                                                    (((0.7981 * cos__q_jA__) * sin__q_jB__) * sin__q_jC__)) *
                                                   sin__q_jE__) +
                                                  ((((0.7981 * sin__q_jA__) * sin__q_jD__) +
                                                       (((((0.7981 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) +
                                                            (((0.7981 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                           cos__q_jD__)) *
                                                      cos__q_jE__)) +
                                                 (((((-0.0316 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                      (((0.0316 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                     sin__q_jD__)) +
                                                ((0.0316 * sin__q_jA__) * cos__q_jD__)) *
                                               cos__q_jF__)) +
                                          (((((((-0.6015 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                 (((0.6015 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                cos__q_jD__) -
                                               ((0.6015 * sin__q_jA__) * sin__q_jD__)) *
                                              sin__q_jE__)) +
                                      (((((0.6015 * cos__q_jA__) * cos__q_jB__) * cos__q_jC__) -
                                           (((0.6015 * cos__q_jA__) * sin__q_jB__) * sin__q_jC__)) *
                                          cos__q_jE__));
        HomogeneousTransform(0, 2) = ((((((((((((0.7447 * cos__q_jA__) * sin__q_jB__) * sin__q_jC__) -
                                                 (((0.7447 * cos__q_jA__) * cos__q_jB__) * cos__q_jC__)) *
                                                sin__q_jE__) +
                                               (((((((-0.7447 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                      (((0.7447 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                     cos__q_jD__) -
                                                    ((0.7447 * sin__q_jA__) * sin__q_jD__)) *
                                                   cos__q_jE__)) +
                                              (((((0.4201 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) +
                                                   (((0.4201 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                  sin__q_jD__)) -
                                             ((0.4201 * sin__q_jA__) * cos__q_jD__)) *
                                            sin__q_jF__) +
                                           (((((((((0.4201 * cos__q_jA__) * sin__q_jB__) * sin__q_jC__) -
                                                    (((0.4201 * cos__q_jA__) * cos__q_jB__) * cos__q_jC__)) *
                                                   sin__q_jE__) +
                                                  (((((((-0.4201 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                         (((0.4201 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                        cos__q_jD__) -
                                                       ((0.4201 * sin__q_jA__) * sin__q_jD__)) *
                                                      cos__q_jE__)) +
                                                 (((((-0.7447 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                      (((0.7447 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                     sin__q_jD__)) +
                                                ((0.7447 * sin__q_jA__) * cos__q_jD__)) *
                                               cos__q_jF__)) +
                                          (((((((-0.5184 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                 (((0.5184 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                cos__q_jD__) -
                                               ((0.5184 * sin__q_jA__) * sin__q_jD__)) *
                                              sin__q_jE__)) +
                                      (((((0.5184 * cos__q_jA__) * cos__q_jB__) * cos__q_jC__) -
                                           (((0.5184 * cos__q_jA__) * sin__q_jB__) * sin__q_jC__)) *
                                          cos__q_jE__));
        HomogeneousTransform(0, 3) = (((((((((((((((0.1808 * cos__q_jA__) * sin__q_jB__) * sin__q_jC__) -
                                                    (((0.1808 * cos__q_jA__) * cos__q_jB__) * cos__q_jC__)) *
                                                   sin__q_jE__) +
                                                  (((((((-0.1808 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                         (((0.1808 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                        cos__q_jD__) -
                                                       ((0.1808 * sin__q_jA__) * sin__q_jD__)) *
                                                      cos__q_jE__)) +
                                                 (((((0.1626 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) +
                                                      (((0.1626 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                     sin__q_jD__)) -
                                                ((0.1626 * sin__q_jA__) * cos__q_jD__)) *
                                               sin__q_jF__) +
                                              (((((((((0.1626 * cos__q_jA__) * sin__q_jB__) * sin__q_jC__) -
                                                       (((0.1626 * cos__q_jA__) * cos__q_jB__) * cos__q_jC__)) *
                                                      sin__q_jE__) +
                                                     (((((((-0.1626 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                            (((0.1626 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                           cos__q_jD__) -
                                                          ((0.1626 * sin__q_jA__) * sin__q_jD__)) *
                                                         cos__q_jE__)) +
                                                    (((((-0.1808 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                         (((0.1808 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                        sin__q_jD__)) +
                                                   ((0.1808 * sin__q_jA__) * cos__q_jD__)) *
                                                  cos__q_jF__)) +
                                             (((((((-0.913 * cos__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                    (((0.913 * cos__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                   cos__q_jD__) -
                                                  ((0.913 * sin__q_jA__) * sin__q_jD__)) *
                                                 sin__q_jE__)) +
                                            (((((0.913 * cos__q_jA__) * cos__q_jB__) * cos__q_jC__) -
                                                 (((0.913 * cos__q_jA__) * sin__q_jB__) * sin__q_jC__)) *
                                                cos__q_jE__)) -
                                           (((0.975 * cos__q_jA__) * sin__q_jB__) * sin__q_jC__)) +
                                          (((0.975 * cos__q_jA__) * cos__q_jB__) * cos__q_jC__)) +
                                      ((0.825 * cos__q_jA__) * sin__q_jB__));
        HomogeneousTransform(1, 0) = ((((((((((((0.6665 * sin__q_jA__) * cos__q_jB__) * cos__q_jC__) -
                                                 (((0.6665 * sin__q_jA__) * sin__q_jB__) * sin__q_jC__)) *
                                                sin__q_jE__) +
                                               (((((((0.6665 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) +
                                                      (((0.6665 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                     cos__q_jD__) -
                                                    ((0.6665 * cos__q_jA__) * sin__q_jD__)) *
                                                   cos__q_jE__)) +
                                              (((((0.4316 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) +
                                                   (((0.4316 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                  sin__q_jD__)) +
                                             ((0.4316 * cos__q_jA__) * cos__q_jD__)) *
                                            sin__q_jF__) +
                                           (((((((((0.4316 * sin__q_jA__) * sin__q_jB__) * sin__q_jC__) -
                                                    (((0.4316 * sin__q_jA__) * cos__q_jB__) * cos__q_jC__)) *
                                                   sin__q_jE__) +
                                                  ((((0.4316 * cos__q_jA__) * sin__q_jD__) +
                                                       (((((-0.4316 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                            (((0.4316 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                           cos__q_jD__)) *
                                                      cos__q_jE__)) +
                                                 (((((0.6665 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) +
                                                      (((0.6665 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                     sin__q_jD__)) +
                                                ((0.6665 * cos__q_jA__) * cos__q_jD__)) *
                                               cos__q_jF__)) +
                                          ((((0.6077 * cos__q_jA__) * sin__q_jD__) +
                                               (((((-0.6077 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                    (((0.6077 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                   cos__q_jD__)) *
                                              sin__q_jE__)) +
                                      (((((0.6077 * sin__q_jA__) * cos__q_jB__) * cos__q_jC__) -
                                           (((0.6077 * sin__q_jA__) * sin__q_jB__) * sin__q_jC__)) *
                                          cos__q_jE__));
        HomogeneousTransform(1, 1) = ((((((((((((0.0316 * sin__q_jA__) * sin__q_jB__) * sin__q_jC__) -
                                                 (((0.0316 * sin__q_jA__) * cos__q_jB__) * cos__q_jC__)) *
                                                sin__q_jE__) +
                                               ((((0.0316 * cos__q_jA__) * sin__q_jD__) +
                                                    (((((-0.0316 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                         (((0.0316 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                        cos__q_jD__)) *
                                                   cos__q_jE__)) +
                                              (((((-0.7981 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                   (((0.7981 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                  sin__q_jD__)) -
                                             ((0.7981 * cos__q_jA__) * cos__q_jD__)) *
                                            sin__q_jF__) +
                                           (((((((((0.7981 * sin__q_jA__) * cos__q_jB__) * cos__q_jC__) -
                                                    (((0.7981 * sin__q_jA__) * sin__q_jB__) * sin__q_jC__)) *
                                                   sin__q_jE__) +
                                                  (((((((0.7981 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) +
                                                         (((0.7981 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                        cos__q_jD__) -
                                                       ((0.7981 * cos__q_jA__) * sin__q_jD__)) *
                                                      cos__q_jE__)) +
                                                 (((((-0.0316 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                      (((0.0316 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                     sin__q_jD__)) -
                                                ((0.0316 * cos__q_jA__) * cos__q_jD__)) *
                                               cos__q_jF__)) +
                                          ((((0.6015 * cos__q_jA__) * sin__q_jD__) +
                                               (((((-0.6015 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                    (((0.6015 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                   cos__q_jD__)) *
                                              sin__q_jE__)) +
                                      (((((0.6015 * sin__q_jA__) * cos__q_jB__) * cos__q_jC__) -
                                           (((0.6015 * sin__q_jA__) * sin__q_jB__) * sin__q_jC__)) *
                                          cos__q_jE__));
        HomogeneousTransform(1, 2) = ((((((((((((0.7447 * sin__q_jA__) * sin__q_jB__) * sin__q_jC__) -
                                                 (((0.7447 * sin__q_jA__) * cos__q_jB__) * cos__q_jC__)) *
                                                sin__q_jE__) +
                                               ((((0.7447 * cos__q_jA__) * sin__q_jD__) +
                                                    (((((-0.7447 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                         (((0.7447 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                        cos__q_jD__)) *
                                                   cos__q_jE__)) +
                                              (((((0.4201 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) +
                                                   (((0.4201 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                  sin__q_jD__)) +
                                             ((0.4201 * cos__q_jA__) * cos__q_jD__)) *
                                            sin__q_jF__) +
                                           (((((((((0.4201 * sin__q_jA__) * sin__q_jB__) * sin__q_jC__) -
                                                    (((0.4201 * sin__q_jA__) * cos__q_jB__) * cos__q_jC__)) *
                                                   sin__q_jE__) +
                                                  ((((0.4201 * cos__q_jA__) * sin__q_jD__) +
                                                       (((((-0.4201 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                            (((0.4201 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                           cos__q_jD__)) *
                                                      cos__q_jE__)) +
                                                 (((((-0.7447 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                      (((0.7447 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                     sin__q_jD__)) -
                                                ((0.7447 * cos__q_jA__) * cos__q_jD__)) *
                                               cos__q_jF__)) +
                                          ((((0.5184 * cos__q_jA__) * sin__q_jD__) +
                                               (((((-0.5184 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                    (((0.5184 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                   cos__q_jD__)) *
                                              sin__q_jE__)) +
                                      (((((0.5184 * sin__q_jA__) * cos__q_jB__) * cos__q_jC__) -
                                           (((0.5184 * sin__q_jA__) * sin__q_jB__) * sin__q_jC__)) *
                                          cos__q_jE__));
        HomogeneousTransform(1, 3) = (((((((((((((((0.1808 * sin__q_jA__) * sin__q_jB__) * sin__q_jC__) -
                                                    (((0.1808 * sin__q_jA__) * cos__q_jB__) * cos__q_jC__)) *
                                                   sin__q_jE__) +
                                                  ((((0.1808 * cos__q_jA__) * sin__q_jD__) +
                                                       (((((-0.1808 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                            (((0.1808 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                           cos__q_jD__)) *
                                                      cos__q_jE__)) +
                                                 (((((0.1626 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) +
                                                      (((0.1626 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                     sin__q_jD__)) +
                                                ((0.1626 * cos__q_jA__) * cos__q_jD__)) *
                                               sin__q_jF__) +
                                              (((((((((0.1626 * sin__q_jA__) * sin__q_jB__) * sin__q_jC__) -
                                                       (((0.1626 * sin__q_jA__) * cos__q_jB__) * cos__q_jC__)) *
                                                      sin__q_jE__) +
                                                     ((((0.1626 * cos__q_jA__) * sin__q_jD__) +
                                                          (((((-0.1626 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                               (((0.1626 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                              cos__q_jD__)) *
                                                         cos__q_jE__)) +
                                                    (((((-0.1808 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                         (((0.1808 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                        sin__q_jD__)) -
                                                   ((0.1808 * cos__q_jA__) * cos__q_jD__)) *
                                                  cos__q_jF__)) +
                                             ((((0.913 * cos__q_jA__) * sin__q_jD__) +
                                                  (((((-0.913 * sin__q_jA__) * cos__q_jB__) * sin__q_jC__) -
                                                       (((0.913 * sin__q_jA__) * sin__q_jB__) * cos__q_jC__)) *
                                                      cos__q_jD__)) *
                                                 sin__q_jE__)) +
                                            (((((0.913 * sin__q_jA__) * cos__q_jB__) * cos__q_jC__) -
                                                 (((0.913 * sin__q_jA__) * sin__q_jB__) * sin__q_jC__)) *
                                                cos__q_jE__)) -
                                           (((0.975 * sin__q_jA__) * sin__q_jB__) * sin__q_jC__)) +
                                          (((0.975 * sin__q_jA__) * cos__q_jB__) * cos__q_jC__)) +
                                      ((0.825 * sin__q_jA__) * sin__q_jB__));
        HomogeneousTransform(2, 0) =
            ((((((((((-0.6665 * cos__q_jB__) * sin__q_jC__) - ((0.6665 * sin__q_jB__) * cos__q_jC__)) * sin__q_jE__) +
                     (((((0.6665 * cos__q_jB__) * cos__q_jC__) - ((0.6665 * sin__q_jB__) * sin__q_jC__)) *
                          cos__q_jD__) *
                         cos__q_jE__)) +
                    ((((0.4316 * cos__q_jB__) * cos__q_jC__) - ((0.4316 * sin__q_jB__) * sin__q_jC__)) * sin__q_jD__)) *
                   sin__q_jF__) +
                  (((((((0.4316 * cos__q_jB__) * sin__q_jC__) + ((0.4316 * sin__q_jB__) * cos__q_jC__)) * sin__q_jE__) +
                        (((((0.4316 * sin__q_jB__) * sin__q_jC__) - ((0.4316 * cos__q_jB__) * cos__q_jC__)) *
                             cos__q_jD__) *
                            cos__q_jE__)) +
                       ((((0.6665 * cos__q_jB__) * cos__q_jC__) - ((0.6665 * sin__q_jB__) * sin__q_jC__)) *
                           sin__q_jD__)) *
                      cos__q_jF__)) +
                 (((((0.6077 * sin__q_jB__) * sin__q_jC__) - ((0.6077 * cos__q_jB__) * cos__q_jC__)) * cos__q_jD__) *
                     sin__q_jE__)) +
                ((((-0.6077 * cos__q_jB__) * sin__q_jC__) - ((0.6077 * sin__q_jB__) * cos__q_jC__)) * cos__q_jE__));
        HomogeneousTransform(2, 1) =
            ((((((((((0.0316 * cos__q_jB__) * sin__q_jC__) + ((0.0316 * sin__q_jB__) * cos__q_jC__)) * sin__q_jE__) +
                     (((((0.0316 * sin__q_jB__) * sin__q_jC__) - ((0.0316 * cos__q_jB__) * cos__q_jC__)) *
                          cos__q_jD__) *
                         cos__q_jE__)) +
                    ((((0.7981 * sin__q_jB__) * sin__q_jC__) - ((0.7981 * cos__q_jB__) * cos__q_jC__)) * sin__q_jD__)) *
                   sin__q_jF__) +
                  (((((((-0.7981 * cos__q_jB__) * sin__q_jC__) - ((0.7981 * sin__q_jB__) * cos__q_jC__)) *
                         sin__q_jE__) +
                        (((((0.7981 * cos__q_jB__) * cos__q_jC__) - ((0.7981 * sin__q_jB__) * sin__q_jC__)) *
                             cos__q_jD__) *
                            cos__q_jE__)) +
                       ((((0.0316 * sin__q_jB__) * sin__q_jC__) - ((0.0316 * cos__q_jB__) * cos__q_jC__)) *
                           sin__q_jD__)) *
                      cos__q_jF__)) +
                 (((((0.6015 * sin__q_jB__) * sin__q_jC__) - ((0.6015 * cos__q_jB__) * cos__q_jC__)) * cos__q_jD__) *
                     sin__q_jE__)) +
                ((((-0.6015 * cos__q_jB__) * sin__q_jC__) - ((0.6015 * sin__q_jB__) * cos__q_jC__)) * cos__q_jE__));
        HomogeneousTransform(2, 2) =
            ((((((((((0.7447 * cos__q_jB__) * sin__q_jC__) + ((0.7447 * sin__q_jB__) * cos__q_jC__)) * sin__q_jE__) +
                     (((((0.7447 * sin__q_jB__) * sin__q_jC__) - ((0.7447 * cos__q_jB__) * cos__q_jC__)) *
                          cos__q_jD__) *
                         cos__q_jE__)) +
                    ((((0.4201 * cos__q_jB__) * cos__q_jC__) - ((0.4201 * sin__q_jB__) * sin__q_jC__)) * sin__q_jD__)) *
                   sin__q_jF__) +
                  (((((((0.4201 * cos__q_jB__) * sin__q_jC__) + ((0.4201 * sin__q_jB__) * cos__q_jC__)) * sin__q_jE__) +
                        (((((0.4201 * sin__q_jB__) * sin__q_jC__) - ((0.4201 * cos__q_jB__) * cos__q_jC__)) *
                             cos__q_jD__) *
                            cos__q_jE__)) +
                       ((((0.7447 * sin__q_jB__) * sin__q_jC__) - ((0.7447 * cos__q_jB__) * cos__q_jC__)) *
                           sin__q_jD__)) *
                      cos__q_jF__)) +
                 (((((0.5184 * sin__q_jB__) * sin__q_jC__) - ((0.5184 * cos__q_jB__) * cos__q_jC__)) * cos__q_jD__) *
                     sin__q_jE__)) +
                ((((-0.5184 * cos__q_jB__) * sin__q_jC__) - ((0.5184 * sin__q_jB__) * cos__q_jC__)) * cos__q_jE__));
        HomogeneousTransform(2, 3) =
            ((((((((((((((0.1808 * cos__q_jB__) * sin__q_jC__) + ((0.1808 * sin__q_jB__) * cos__q_jC__)) *
                          sin__q_jE__) +
                         (((((0.1808 * sin__q_jB__) * sin__q_jC__) - ((0.1808 * cos__q_jB__) * cos__q_jC__)) *
                              cos__q_jD__) *
                             cos__q_jE__)) +
                        ((((0.1626 * cos__q_jB__) * cos__q_jC__) - ((0.1626 * sin__q_jB__) * sin__q_jC__)) *
                            sin__q_jD__)) *
                       sin__q_jF__) +
                      (((((((0.1626 * cos__q_jB__) * sin__q_jC__) + ((0.1626 * sin__q_jB__) * cos__q_jC__)) *
                             sin__q_jE__) +
                            (((((0.1626 * sin__q_jB__) * sin__q_jC__) - ((0.1626 * cos__q_jB__) * cos__q_jC__)) *
                                 cos__q_jD__) *
                                cos__q_jE__)) +
                           ((((0.1808 * sin__q_jB__) * sin__q_jC__) - ((0.1808 * cos__q_jB__) * cos__q_jC__)) *
                               sin__q_jD__)) *
                          cos__q_jF__)) +
                     (((((0.913 * sin__q_jB__) * sin__q_jC__) - ((0.913 * cos__q_jB__) * cos__q_jC__)) * cos__q_jD__) *
                         sin__q_jE__)) +
                    ((((-0.913 * cos__q_jB__) * sin__q_jC__) - ((0.913 * sin__q_jB__) * cos__q_jC__)) * cos__q_jE__)) -
                   ((0.975 * cos__q_jB__) * sin__q_jC__)) -
                  ((0.975 * sin__q_jB__) * cos__q_jC__)) +
                 (0.825 * cos__q_jB__)) +
                0.25);

        // end of copied part

        return HomogeneousTransform;
    }


    Eigen::Matrix<SCALAR, 3, 1> get_position(const Eigen::Matrix<SCALAR, 6, 1>& q)
    {
        Eigen::Matrix<SCALAR, 4, 4> HomogeneousTransform = getHomogeneousTransform(q);

        return HomogeneousTransform.block(0, 3, 3, 1);
    }
};


}  // namespace example
}  // namespace optcon
}  // namespace ct

#endif
