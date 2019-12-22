/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-value"


#include <memory>
#include <array>

#include <Eigen/Dense>

#include <gtest/gtest.h>

#include <ct/optcon/optcon.h>

#include <ct/rbd/robot/jacobian/OperationalJacobianBase.h>
#include <ct/rbd/operationalSpace/rigid_body/OperationalModel.h>
#include <ct/rbd/operationalSpace/rigid_body/OperationalModelRBD.h>
#include "../models/testhyq/RobCoGenTestHyQ.h"

using namespace ct::rbd;


class TestJacobian : public OperationalJacobianBase<3, 12>
{
public:
    typedef ct::rbd::OperationalJacobianBase<3, 12> Base;
    typedef typename Base::state_t state_t;
    typedef typename Base::jacobian_t origin_jacobian_t;

    TestJacobian() {}
    ~TestJacobian() {}
    void getJacobianOrigin(const state_t& state, origin_jacobian_t& J) override { J.setIdentity(); }
    void getJacobianOriginDerivative(const state_t& state, origin_jacobian_t& dJdt) override { dJdt.setZero(); }
};


TEST(OperationalSpaceTest, OperationalSpaceTest)
{
    size_t nTests = 100;

    for (size_t i = 0; i < nTests; i++)
    {
        typedef Eigen::Matrix<double, 18, 1> robot_coordinate_t;
        typedef Eigen::Matrix<double, 3, 1> operational_coordinate_t;
        typedef typename OperationalModel<3, 12, 0>::state_t state_t;

        // pointer to the test robot RBDContainer
        std::shared_ptr<TestHyQ::RobCoGenContainer> testRbdContainerPtr(new TestHyQ::RobCoGenContainer());
        // end-effector array for the possible contact points
        std::array<EndEffector<12>, 0> EndEffectorArray;

        // test robot model
        OperationalModelRBD<TestHyQ::RobCoGenContainer, 0>::ptr testRobotModel(
            new OperationalModelRBD<TestHyQ::RobCoGenContainer, 0>(testRbdContainerPtr, EndEffectorArray));
        // test robot Jacobian
        TestJacobian::ptr testJacobian(new TestJacobian());
        // operational space
        OperationalModel<3, 12, 0> testOperationalModel(testRobotModel, testJacobian);

        Eigen::Matrix<double, 12, 1> tau = Eigen::Matrix<double, 12, 1>::Random();
        state_t state;
        state.fromStateVectorEulerXyz(state_t::state_vector_euler_t::Random());
        //robot_coordinate_t q  = state.toCoordinatePosition();
        //robot_coordinate_t qd = state.toCoordinateVelocity();

        // update the robot model
        testRobotModel->update(state);
        robot_coordinate_t qdd;
        qdd = testRobotModel->MInverse() *
              (-testRobotModel->C() - testRobotModel->G() + testRobotModel->S().transpose() * tau);

        // update the operational space model
        testOperationalModel.update(state);
        operational_coordinate_t xdd = testOperationalModel.getAccelerations(qdd);

        operational_coordinate_t testModelError = testOperationalModel.M() * xdd + testOperationalModel.C() +
                                                  testOperationalModel.G() - testOperationalModel.S().transpose() * tau;

        double maxError = testModelError.cwiseAbs().maxCoeff();

        ASSERT_LT(maxError, 1e-4);
    }
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


#pragma GCC diagnostic pop
