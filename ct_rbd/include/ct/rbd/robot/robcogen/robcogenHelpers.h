/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#ifdef CT_L23
#error You specified 23 links but the maximum supported is 22.
#endif

#ifdef CT_EE23
#error You specified 23 end-effectors but the maximum supported is 22.
#endif

#ifndef ROBCOGEN_NS
#error Please specify the NS of the robot in RobCoGen (ROBCOGEN_NS)
#endif

#ifndef TARGET_NS
#error Please specify a target NS (TARGET_NS)
#endif

#ifndef CT_BASE
#error Please specify base name (CT_BASE)
#endif

#ifndef CT_N_EE
#error Please specify the number of endeffectors (CT_N_EE)
#endif

#define CT_RBD_TRANSFORM_BASE_TO_ID(BASE, LINK_ID) transforms.BASE##_X_##LINK_ID

#define CT_RBD_JACOBIAN_BASE_TO_ID(BASE, EE_ID) jacobians.BASE##_J_##EE_ID

#define CT_RBD_JACOBIAN_GET_BLOCK(FIRST, LAST) jacobian.template block<6, LAST - FIRST + 1>(0, FIRST)

// This is just a helper Macro that generates a case-statement for each link to shorten the macro below.
#define CT_RBD_CASE_HELPER_BASE_ID(BASE, LINK_ID, INDEX)   \
    case INDEX:                                            \
        return CT_RBD_TRANSFORM_BASE_TO_ID(BASE, LINK_ID); \
        break;

// This is just a helper Macro that generates a case-statement for each link to shorten the macro below.
#define CT_RBD_CASE_HELPER_JAC_UPDATE(BASE, LINK_ID) CT_RBD_JACOBIAN_BASE_TO_ID(BASE, LINK_ID).update(q);

// This is just a helper Macro that generates a case-statement for each link to shorten the macro below.
#define CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(BASE, LINK_ID, INDEX) \
    case INDEX:                                                      \
        return CT_RBD_TRANSFORM_BASE_TO_ID(BASE, LINK_ID).update(

#define CT_RBD_TRANSFORM_ID_TO_BASE(BASE, LINK_ID) transforms.LINK_ID##_X_##BASE

// This is just a helper Macro that generates a case-statement for each link to shorten the macro below.
#define CT_RBD_CASE_HELPER_ID_BASE(BASE, LINK_ID, INDEX)   \
    case INDEX:                                            \
        return CT_RBD_TRANSFORM_ID_TO_BASE(BASE, LINK_ID); \
        break;

// This is just a helper Macro that generates a case-statement for each link to shorten the macro below.
#define CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(BASE, LINK_ID, INDEX) \
    case INDEX:                                                      \
        return CT_RBD_TRANSFORM_ID_TO_BASE(BASE, LINK_ID).update(

#define CT_RBD_CASE_HELPER_JOINT_END \
        ) ;                          \
    break;


namespace ct {
namespace rbd {
namespace TARGET_NS {

template <typename SCALAR>
class Utils
{
private:
    static const int NJOINTS = iit::ROBCOGEN_NS::tpl::Traits<SCALAR>::joints_count;

public:
    static const int N_EE = CT_N_EE;

    // This defines a function to get the transform by ID
    template <class TRANS>
    static typename TRANS::MatrixType getTransformBaseLinkById(TRANS& transforms,
        size_t link_id,
        const Eigen::Matrix<SCALAR, NJOINTS, 1>& q)
    {
        switch (link_id)
        {
            case 0:
            {
                typename TRANS::MatrixType identity;
                identity.setZero();
                identity.template topRightCorner<3, 3>().setIdentity();
                return identity;
                break;
            }
#ifdef CT_L0
                CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(CT_BASE, CT_L0, 0 + 1)
                q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L1
                        CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(CT_BASE, CT_L1, 1 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L2
                            CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(CT_BASE, CT_L2, 2 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L3
                                CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                    CT_BASE, CT_L3, 3 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L4
                                    CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                        CT_BASE, CT_L4, 4 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L5
                                        CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                            CT_BASE, CT_L5, 5 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L6
                                            CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                CT_BASE, CT_L6, 6 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L7
                                                CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                    CT_BASE, CT_L7, 7 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L8
                                                    CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                        CT_BASE, CT_L8, 8 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L9
                                                        CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                            CT_BASE, CT_L9, 9 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L10
                                                            CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                CT_BASE, CT_L10, 10 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L11
                                                                CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(CT_BASE, CT_L11,
                                                                    11 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L12
                                                                    CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(CT_BASE,
                                                                        CT_L12, 12 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L13
                                                                        CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(CT_BASE,
                                                                            CT_L13,
                                                                            13 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L14
                                                                            CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                                CT_BASE, CT_L14,
                                                                                14 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L15
                                                                                CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                                    CT_BASE, CT_L15,
                                                                                    15 +
                                                                                        1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L16
                                                                                    CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                                        CT_BASE, CT_L16,
                                                                                        16 +
                                                                                            1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L17
                                                                                        CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                                            CT_BASE, CT_L17,
                                                                                            17 +
                                                                                                1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L18
                                                                                            CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                                                CT_BASE, CT_L18,
                                                                                                18 +
                                                                                                    1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L19
                                                                                                CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                                                    CT_BASE, CT_L19,
                                                                                                    19 +
                                                                                                        1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L20
                                                                                                    CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                                                        CT_BASE, CT_L20,
                                                                                                        20 +
                                                                                                            1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L21
                                                                                                        CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                                                            CT_BASE,
                                                                                                            CT_L21,
                                                                                                            21 +
                                                                                                                1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L22
                                                                                                            CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                                                                CT_BASE,
                                                                                                                CT_L22,
                                                                                                                22 +
                                                                                                                    1) q
                                                                                                                CT_RBD_CASE_HELPER_JOINT_END
#endif
                    default : std::cout
                              << "getTransformByLinkID: requested joint does not exist, requested: "
                              << link_id
                              << std::endl;
                throw std::runtime_error("getTransformByLinkID: requested joint does not exist");
                break;
        }
    }


    // This defines a function to get the transform by ID
    template <class TRANS>
    static typename TRANS::MatrixType getTransformLinkBaseById(TRANS& transforms,
        size_t link_id,
        const Eigen::Matrix<SCALAR, NJOINTS, 1>& q)
    {
        switch (link_id)
        {
            case 0:
            {
                typename TRANS::MatrixType identity;
                identity.setZero();
                identity.template topRightCorner<3, 3>().setIdentity();
                return identity;
                break;
            }
#ifdef CT_L0
                CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(CT_BASE, CT_L0, 0 + 1)
                q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L1
                        CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(CT_BASE, CT_L1, 1 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L2
                            CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(CT_BASE, CT_L2, 2 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L3
                                CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(
                                    CT_BASE, CT_L3, 3 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L4
                                    CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(
                                        CT_BASE, CT_L4, 4 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L5
                                        CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(
                                            CT_BASE, CT_L5, 5 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L6
                                            CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(
                                                CT_BASE, CT_L6, 6 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L7
                                                CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(
                                                    CT_BASE, CT_L7, 7 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L8
                                                    CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(
                                                        CT_BASE, CT_L8, 8 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L9
                                                        CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(
                                                            CT_BASE, CT_L9, 9 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L10
                                                            CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(
                                                                CT_BASE, CT_L10, 10 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L11
                                                                CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(CT_BASE, CT_L11,
                                                                    11 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L12
                                                                    CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(CT_BASE,
                                                                        CT_L12, 12 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L13
                                                                        CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(CT_BASE,
                                                                            CT_L13,
                                                                            13 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L14
                                                                            CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(
                                                                                CT_BASE, CT_L14,
                                                                                14 + 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L15
                                                                                CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(
                                                                                    CT_BASE, CT_L15,
                                                                                    15 +
                                                                                        1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L16
                                                                                    CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(
                                                                                        CT_BASE, CT_L16,
                                                                                        16 +
                                                                                            1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L17
                                                                                        CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(
                                                                                            CT_BASE, CT_L17,
                                                                                            17 +
                                                                                                1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L18
                                                                                            CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(
                                                                                                CT_BASE, CT_L18,
                                                                                                18 +
                                                                                                    1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L19
                                                                                                CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(
                                                                                                    CT_BASE, CT_L19,
                                                                                                    19 +
                                                                                                        1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L20
                                                                                                    CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(
                                                                                                        CT_BASE, CT_L20,
                                                                                                        20 +
                                                                                                            1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L21
                                                                                                        CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(
                                                                                                            CT_BASE,
                                                                                                            CT_L21,
                                                                                                            21 +
                                                                                                                1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_L22
                                                                                                            CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE(
                                                                                                                CT_BASE,
                                                                                                                CT_L22,
                                                                                                                22 +
                                                                                                                    1) q
                                                                                                                CT_RBD_CASE_HELPER_JOINT_END
#endif
                    default : std::cout
                              << "getTransformByLinkID: requested joint does not exist, requested: "
                              << link_id
                              << std::endl;
                throw std::runtime_error("getTransformByLinkID: requested joint does not exist");
                break;
        }
    }


    // This defines a function to get the transform by ID
    template <class TRANS>
    static typename TRANS::MatrixType getTransformBaseEEById(TRANS& transforms,
        size_t ee_id,
        const Eigen::Matrix<SCALAR, NJOINTS, 1>& q)
    {
        switch (ee_id)
        {
#ifdef CT_EE0
            CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(CT_BASE, CT_EE0, 0)
            q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE1
                    CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(CT_BASE, CT_EE1, 1) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE2
                        CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(CT_BASE, CT_EE2, 2) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE3
                            CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(CT_BASE, CT_EE3, 3) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE4
                                CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                    CT_BASE, CT_EE4, 4) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE5
                                    CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                        CT_BASE, CT_EE5, 5) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE6
                                        CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                            CT_BASE, CT_EE6, 6) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE7
                                            CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                CT_BASE, CT_EE7, 7) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE8
                                                CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                    CT_BASE, CT_EE8, 8) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE9
                                                    CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                        CT_BASE, CT_EE9, 9) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE10
                                                        CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                            CT_BASE, CT_EE10, 10) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE11
                                                            CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                CT_BASE, CT_EE11, 11) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE12
                                                                CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                    CT_BASE, CT_EE12, 12) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE13
                                                                    CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(CT_BASE,
                                                                        CT_EE13, 13) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE14
                                                                        CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(CT_BASE,
                                                                            CT_EE14, 14) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE15
                                                                            CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                                CT_BASE, CT_EE15,
                                                                                15) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE16
                                                                                CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                                    CT_BASE, CT_EE16,
                                                                                    16) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE17
                                                                                    CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                                        CT_BASE, CT_EE17,
                                                                                        17) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE18
                                                                                        CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                                            CT_BASE, CT_EE18,
                                                                                            18) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE19
                                                                                            CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                                                CT_BASE, CT_EE19,
                                                                                                19) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE20
                                                                                                CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                                                    CT_BASE, CT_EE20,
                                                                                                    20) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE21
                                                                                                    CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                                                        CT_BASE,
                                                                                                        CT_EE21,
                                                                                                        21) q CT_RBD_CASE_HELPER_JOINT_END
#endif
#ifdef CT_EE22
                                                                                                        CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID(
                                                                                                            CT_BASE,
                                                                                                            CT_EE22,
                                                                                                            22) q
                                                                                                            CT_RBD_CASE_HELPER_JOINT_END
#endif
                default : std::cout
                          << "getTransformByEEID: requested end-effector does not exist, requested: "
                          << ee_id
                          << std::endl;
            throw std::runtime_error("getTransformByEEID: requested end-effector does not exist");
            break;
        }
    }

    static size_t eeIdToLinkId(size_t ee_id)
    {
        switch (ee_id)
        {
#ifdef CT_EE0
            case 0:
                return CT_EE0_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE1
            case 1:
                return CT_EE1_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE2
            case 2:
                return CT_EE2_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE3
            case 3:
                return CT_EE3_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE4
            case 4:
                return CT_EE4_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE5
            case 5:
                return CT_EE5_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE6
            case 6:
                return CT_EE6_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE7
            case 7:
                return CT_EE7_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE8
            case 8:
                return CT_EE8_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE9
            case 9:
                return CT_EE9_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE10
            case 10:
                return CT_EE10_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE11
            case 11:
                return CT_EE11_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE12
            case 12:
                return CT_EE12_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE13
            case 13:
                return CT_EE13_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE14
            case 14:
                return CT_EE14_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE15
            case 15:
                return CT_EE15_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE16
            case 16:
                return CT_EE16_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE17
            case 17:
                return CT_EE17_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE18
            case 18:
                return CT_EE18_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE19
            case 19:
                return CT_EE19_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE20
            case 20:
                return CT_EE20_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE21
            case 21:
                return CT_EE21_IS_ON_LINK;
                break;
#endif
#ifdef CT_EE22
            case 22:
                return CT_EE22_IS_ON_LINK;
                break;
#endif

            default:
                std::cout << "eeIdToLinkId: requested end-effector does not exist, requested: " << ee_id << std::endl;
                throw std::runtime_error("getTransformByEEID: requested end-effector does not exist");
                break;
        }
    }


    // This defines a function to get the transform by ID
    template <class JACS>
    static typename Eigen::Matrix<SCALAR, 6, NJOINTS> getJacobianBaseEEbyId(JACS& jacobians,
        size_t ee_id,
        const Eigen::Matrix<SCALAR, NJOINTS, 1>& q)
    {
        Eigen::Matrix<SCALAR, 6, NJOINTS> jacobian(Eigen::Matrix<SCALAR, 6, NJOINTS>::Zero());

        switch (ee_id)
        {
#ifdef CT_EE0
            case 0:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE0_FIRST_JOINT, CT_EE0_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE0);
                break;
#endif
#ifdef CT_EE1
            case 1:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE1_FIRST_JOINT, CT_EE1_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE1);
                break;
#endif
#ifdef CT_EE2
            case 2:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE2_FIRST_JOINT, CT_EE2_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE2);
                break;
#endif
#ifdef CT_EE3
            case 3:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE3_FIRST_JOINT, CT_EE3_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE3);
                break;
#endif
#ifdef CT_EE4
            case 4:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE4_FIRST_JOINT, CT_EE4_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE4);
                break;
#endif
#ifdef CT_EE5
            case 5:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE5_FIRST_JOINT, CT_EE5_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE5);
                break;
#endif
#ifdef CT_EE6
            case 6:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE6_FIRST_JOINT, CT_EE6_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE6);
                break;
#endif
#ifdef CT_EE7
            case 7:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE7_FIRST_JOINT, CT_EE7_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE7);
                break;
#endif
#ifdef CT_EE8
            case 8:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE8_FIRST_JOINT, CT_EE8_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE8);
                break;
#endif
#ifdef CT_EE9
            case 9:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE9_FIRST_JOINT, CT_EE9_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE9);
                break;
#endif
#ifdef CT_EE10
            case 10:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE10_FIRST_JOINT, CT_EE10_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE10);
                break;
#endif
#ifdef CT_EE11
            case 11:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE11_FIRST_JOINT, CT_EE11_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE11);
                break;
#endif
#ifdef CT_EE12
            case 12:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE12_FIRST_JOINT, CT_EE12_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE12);
                break;
#endif
#ifdef CT_EE13
            case 13:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE13_FIRST_JOINT, CT_EE13_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE13);
                break;
#endif
#ifdef CT_EE14
            case 14:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE14_FIRST_JOINT, CT_EE14_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE14);
                break;
#endif
#ifdef CT_EE15
            case 15:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE15_FIRST_JOINT, CT_EE15_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE15);
                break;
#endif
#ifdef CT_EE16
            case 16:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE16_FIRST_JOINT, CT_EE16_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE16);
                break;
#endif
#ifdef CT_EE17
            case 17:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE17_FIRST_JOINT, CT_EE17_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE17);
                break;
#endif
#ifdef CT_EE18
            case 18:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE18_FIRST_JOINT, CT_EE18_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE18);
                break;
#endif
#ifdef CT_EE19
            case 19:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE19_FIRST_JOINT, CT_EE19_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE19);
                break;
#endif
#ifdef CT_EE20
            case 20:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE20_FIRST_JOINT, CT_EE20_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE20);
                break;
#endif
#ifdef CT_EE21
            case 21:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE21_FIRST_JOINT, CT_EE21_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE21);
                break;
#endif
#ifdef CT_EE22
            case 22:
                CT_RBD_JACOBIAN_GET_BLOCK(CT_EE22_FIRST_JOINT, CT_EE22_LAST_JOINT) =
                    CT_RBD_CASE_HELPER_JAC_UPDATE(CT_BASE, CT_EE22);
                break;
#endif

            default:
                std::cout << "getJacobianBaseByEEID: requested end-effector does not exist, requested: " << ee_id
                          << std::endl;
                throw std::runtime_error("getJacobianBaseByEEID: requested end-effector does not exist");
                break;
        }

        return jacobian;
    }

};  // class Utils


namespace tpl {

template <typename SCALAR>
using RobCoGenContainer =
    ct::rbd::RobCoGenContainer<iit::ROBCOGEN_NS::tpl::Traits<SCALAR>, iit::ROBCOGEN_NS::LinkDataMap, Utils<SCALAR>>;

template <typename SCALAR>
using Kinematics = ct::rbd::Kinematics<RobCoGenContainer<SCALAR>, Utils<SCALAR>::N_EE>;

template <typename SCALAR>
using Dynamics = ct::rbd::Dynamics<RobCoGenContainer<SCALAR>, Utils<SCALAR>::N_EE>;


}  // namespace tpl


typedef tpl::RobCoGenContainer<double> RobCoGenContainer;
typedef tpl::Kinematics<double> Kinematics;
typedef Dynamics<RobCoGenContainer, Kinematics::NUM_EE> Dynamics;

}  // TARGET_NS


}  // namespace rbd

}  // namespace ct

// CLEAN UP

#undef CT_BASE
#undef ROBCOGEN_NS
#undef ROBOT_NAME
#undef TARGET_NS
#undef CT_N_EE

#undef CT_L0
#undef CT_L1
#undef CT_L2
#undef CT_L3
#undef CT_L4
#undef CT_L5
#undef CT_L6
#undef CT_L7
#undef CT_L8
#undef CT_L9
#undef CT_L10
#undef CT_L11
#undef CT_L12
#undef CT_L13
#undef CT_L14
#undef CT_L15
#undef CT_L16
#undef CT_L17
#undef CT_L18
#undef CT_L19
#undef CT_L20
#undef CT_L21
#undef CT_L22

#undef CT_EE0
#undef CT_EE1
#undef CT_EE2
#undef CT_EE3
#undef CT_EE4
#undef CT_EE5
#undef CT_EE6
#undef CT_EE7
#undef CT_EE8
#undef CT_EE9
#undef CT_EE10
#undef CT_EE11
#undef CT_EE12
#undef CT_EE13
#undef CT_EE14
#undef CT_EE15
#undef CT_EE16
#undef CT_EE17
#undef CT_EE18
#undef CT_EE19
#undef CT_EE20
#undef CT_EE21
#undef CT_EE22

#undef CT_EE0_IS_ON_LINK
#undef CT_EE1_IS_ON_LINK
#undef CT_EE2_IS_ON_LINK
#undef CT_EE3_IS_ON_LINK
#undef CT_EE4_IS_ON_LINK
#undef CT_EE5_IS_ON_LINK
#undef CT_EE6_IS_ON_LINK
#undef CT_EE7_IS_ON_LINK
#undef CT_EE8_IS_ON_LINK
#undef CT_EE9_IS_ON_LINK
#undef CT_EE10_IS_ON_LINK
#undef CT_EE11_IS_ON_LINK
#undef CT_EE12_IS_ON_LINK
#undef CT_EE13_IS_ON_LINK
#undef CT_EE14_IS_ON_LINK
#undef CT_EE15_IS_ON_LINK
#undef CT_EE16_IS_ON_LINK
#undef CT_EE17_IS_ON_LINK
#undef CT_EE18_IS_ON_LINK
#undef CT_EE19_IS_ON_LINK
#undef CT_EE20_IS_ON_LINK
#undef CT_EE21_IS_ON_LINK
#undef CT_EE22_IS_ON_LINK

#undef CT_EE0_FIRST_JOINT
#undef CT_EE1_FIRST_JOINT
#undef CT_EE2_FIRST_JOINT
#undef CT_EE3_FIRST_JOINT
#undef CT_EE4_FIRST_JOINT
#undef CT_EE5_FIRST_JOINT
#undef CT_EE6_FIRST_JOINT
#undef CT_EE7_FIRST_JOINT
#undef CT_EE8_FIRST_JOINT
#undef CT_EE9_FIRST_JOINT
#undef CT_EE10_FIRST_JOINT
#undef CT_EE11_FIRST_JOINT
#undef CT_EE12_FIRST_JOINT
#undef CT_EE13_FIRST_JOINT
#undef CT_EE14_FIRST_JOINT
#undef CT_EE15_FIRST_JOINT
#undef CT_EE16_FIRST_JOINT
#undef CT_EE17_FIRST_JOINT
#undef CT_EE18_FIRST_JOINT
#undef CT_EE19_FIRST_JOINT
#undef CT_EE20_FIRST_JOINT
#undef CT_EE21_FIRST_JOINT
#undef CT_EE22_FIRST_JOINT


#undef CT_EE0_LAST_JOINT
#undef CT_EE1_LAST_JOINT
#undef CT_EE2_LAST_JOINT
#undef CT_EE3_LAST_JOINT
#undef CT_EE4_LAST_JOINT
#undef CT_EE5_LAST_JOINT
#undef CT_EE6_LAST_JOINT
#undef CT_EE7_LAST_JOINT
#undef CT_EE8_LAST_JOINT
#undef CT_EE9_LAST_JOINT
#undef CT_EE10_LAST_JOINT
#undef CT_EE11_LAST_JOINT
#undef CT_EE12_LAST_JOINT
#undef CT_EE13_LAST_JOINT
#undef CT_EE14_LAST_JOINT
#undef CT_EE15_LAST_JOINT
#undef CT_EE16_LAST_JOINT
#undef CT_EE17_LAST_JOINT
#undef CT_EE18_LAST_JOINT
#undef CT_EE19_LAST_JOINT
#undef CT_EE20_LAST_JOINT
#undef CT_EE21_LAST_JOINT
#undef CT_EE22_LAST_JOINT


#undef CT_RBD_TRANSFORM_BASE_TO_ID
#undef CT_RBD_JACOBIAN_BASE_TO_ID
#undef CT_RBD_JACOBIAN_GET_BLOCK
#undef CT_RBD_CASE_HELPER_BASE_ID
#undef CT_RBD_CASE_HELPER_JAC_UPDATE
#undef CT_RBD_CASE_HELPER_JOINT_BEGIN_BASE_ID
#undef CT_RBD_TRANSFORM_ID_TO_BASE
#undef CT_RBD_CASE_HELPER_ID_BASE
#undef CT_RBD_CASE_HELPER_JOINT_BEGIN_ID_BASE
#undef CT_RBD_CASE_HELPER_JOINT_END
