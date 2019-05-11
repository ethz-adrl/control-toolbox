/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * This unit test for the box constraint terms compares automatically generated sparsity patterns against hard-coded ones.
 */

#include <ct/optcon/optcon.h>
#include <gtest/gtest.h>

using namespace ct::core;
using namespace ct::optcon;

// state and control dimensions
const size_t state_dim = 10;
const size_t control_dim = 5;

bool verbose = false;

TEST(SparseBoxConstraintTest, HardcodedPatternTest)
{
    // create constraint container
    std::shared_ptr<ConstraintContainerAnalytical<state_dim, control_dim>> constraints(
        new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());

    // box constraint boundaries with sparsities
    Eigen::VectorXi sp_control(control_dim);
    sp_control << 0, 1, 0, 0, 1;
    Eigen::VectorXd u_lb(2);
    Eigen::VectorXd u_ub(2);
    u_lb.setConstant(-1.11);
    u_ub = -u_lb;

    Eigen::VectorXi sp_state(state_dim);
    sp_state << 0, 1, 0, 0, 1, 0, 1, 1, 0, 0;
    Eigen::VectorXd x_lb(4);
    Eigen::VectorXd x_ub(4);
    x_lb.setConstant(-3.33);
    x_ub = -x_lb;

    // constraint terms
    std::shared_ptr<ControlInputConstraint<state_dim, control_dim>> controlConstraint(
        new ControlInputConstraint<state_dim, control_dim>(u_lb, u_ub, sp_control));
    controlConstraint->setName("ControlInputConstraint");
    std::shared_ptr<StateConstraint<state_dim, control_dim>> stateConstraint(
        new StateConstraint<state_dim, control_dim>(x_lb, x_ub, sp_state));
    stateConstraint->setName("StateConstraint");

    // add and initialize constraint terms
    constraints->addIntermediateConstraint(controlConstraint, true);
    constraints->addIntermediateConstraint(stateConstraint, true);
    constraints->initialize();

    if (verbose)
    {
        std::cout << "=============================================" << std::endl;
        std::cout << "Printing example for sparse box constraint:" << std::endl;
        std::cout << "=============================================" << std::endl;
        constraints->printout();
    }


    /*!
     * compare results against hard-coded references.
     */

    ASSERT_EQ(constraints->getJacobianStateNonZeroCountIntermediate(), 4);
    ASSERT_EQ(constraints->getJacobianInputNonZeroCountIntermediate(), 2);

    Eigen::Matrix<double, 4 + 2, state_dim> jac_x_ref = Eigen::Matrix<double, 4 + 2, state_dim>::Zero();
    jac_x_ref(2, 1) = 1;
    jac_x_ref(3, 4) = 1;
    jac_x_ref(4, 6) = 1;
    jac_x_ref(5, 7) = 1;
    Eigen::MatrixXd jac_x = constraints->jacobianStateIntermediate();
    for (size_t i = 0; i < 4 + 2; i++)
        for (size_t j = 0; j < state_dim; j++)
            ASSERT_EQ(jac_x(i, j), jac_x_ref(i, j));


    Eigen::Matrix<double, 4 + 2, control_dim> jac_u_ref = Eigen::Matrix<double, 4 + 2, control_dim>::Zero();
    jac_u_ref(0, 1) = 1;
    jac_u_ref(1, 4) = 1;
    Eigen::MatrixXd jac_u = constraints->jacobianInputIntermediate();
    for (size_t i = 0; i < 4 + 2; i++)
        for (size_t j = 0; j < control_dim; j++)
            ASSERT_EQ(jac_u(i, j), jac_u_ref(i, j));


    ASSERT_EQ(constraints->jacobianInputSparseIntermediate().rows(), 2);
    ASSERT_EQ(constraints->jacobianStateSparseIntermediate().rows(), 4);

    Eigen::VectorXi iRows, jCols;

    constraints->sparsityPatternStateIntermediate(iRows, jCols);
    ASSERT_EQ(iRows(0), 2);
    ASSERT_EQ(iRows(1), 3);
    ASSERT_EQ(iRows(2), 4);
    ASSERT_EQ(iRows(3), 5);
    ASSERT_EQ(jCols(0), 1);
    ASSERT_EQ(jCols(1), 4);
    ASSERT_EQ(jCols(2), 6);
    ASSERT_EQ(jCols(3), 7);

    constraints->sparsityPatternInputIntermediate(iRows, jCols);
    ASSERT_EQ(iRows(0), 0);
    ASSERT_EQ(iRows(1), 1);
    ASSERT_EQ(jCols(0), 1);
    ASSERT_EQ(jCols(1), 4);
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
