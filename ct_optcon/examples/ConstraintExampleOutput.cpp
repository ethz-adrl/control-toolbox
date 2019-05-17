/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * This Example displays some constraint toolbox outputs, giving the user a feeling for
 * sparsity patterns, etc.
 */

#include <ct/optcon/optcon.h>

using namespace ct::core;
using namespace ct::optcon;

// some random state and control dimensions
const size_t state_dim = 10;
const size_t control_dim = 5;


void controlInputBoxConstraintExample()
{
    // create constraint container
    std::shared_ptr<ConstraintContainerAnalytical<state_dim, control_dim>> constraints(
        new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());

    // desired boundaries
    ControlVector<control_dim> u_lb = -ControlVector<control_dim>::Ones();
    ControlVector<control_dim> u_ub = ControlVector<control_dim>::Ones();

    // constraint term
    std::shared_ptr<ControlInputConstraint<state_dim, control_dim>> controlConstraint(
        new ControlInputConstraint<state_dim, control_dim>(u_lb, u_ub));
    controlConstraint->setName("ControlInputConstraint");

    // add and initialize constraint term
    constraints->addIntermediateConstraint(controlConstraint, true);
    constraints->initialize();

    std::cout << "=============================================" << std::endl;
    std::cout << "Printing example for control input constraint:" << std::endl;
    std::cout << "=============================================" << std::endl;
    constraints->printout();
}


void terminalConstraintExample()
{
    // create constraint container
    std::shared_ptr<ConstraintContainerAnalytical<state_dim, control_dim>> constraints(
        new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());

    // desired terminal state
    StateVector<state_dim> x_final = StateVector<state_dim>::Random();

    // terminal constrain term
    std::shared_ptr<TerminalConstraint<state_dim, control_dim>> terminalConstraint(
        new TerminalConstraint<state_dim, control_dim>(x_final));
    terminalConstraint->setName("TerminalConstraint");

    // add and initialize terminal constraint term
    constraints->addTerminalConstraint(terminalConstraint, true);
    constraints->initialize();

    std::cout << "==============================================" << std::endl;
    std::cout << "Printing example for terminal state constraint:" << std::endl;
    std::cout << "==============================================" << std::endl;
    constraints->printout();
}


void boxConstraintsExample()
{
    // create constraint container
    std::shared_ptr<ConstraintContainerAnalytical<state_dim, control_dim>> constraints(
        new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());

    // desired terminal state
    ControlVector<control_dim> u_lb = -1.11 * ControlVector<control_dim>::Ones();
    ControlVector<control_dim> u_ub = 1.11 * ControlVector<control_dim>::Ones();
    StateVector<state_dim> x_lb = -3.33 * StateVector<state_dim>::Ones();
    StateVector<state_dim> x_ub = 3.33 * StateVector<state_dim>::Ones();

    // constrain terms
    std::shared_ptr<ControlInputConstraint<state_dim, control_dim>> controlConstraint(
        new ControlInputConstraint<state_dim, control_dim>(u_lb, u_ub));
    controlConstraint->setName("ControlInputConstraint");
    std::shared_ptr<StateConstraint<state_dim, control_dim>> stateConstraint(
        new StateConstraint<state_dim, control_dim>(x_lb, x_ub));
    stateConstraint->setName("StateConstraint");

    // add and initialize constraint terms
    constraints->addIntermediateConstraint(controlConstraint, true);
    constraints->addIntermediateConstraint(stateConstraint, true);
    constraints->initialize();

    std::cout << "=============================================" << std::endl;
    std::cout << "Printing example for combined box constraint:" << std::endl;
    std::cout << "=============================================" << std::endl;
    constraints->printout();
}


void sparseBoxConstraintsExample()
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

    // constrain terms
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

    std::cout << "=============================================" << std::endl;
    std::cout << "Printing example for sparse box constraint:" << std::endl;
    std::cout << "=============================================" << std::endl;
    constraints->printout();
}


int main(int argc, char** argv)
{
    controlInputBoxConstraintExample();
    terminalConstraintExample();
    boxConstraintsExample();
    sparseBoxConstraintsExample();
    return 1;
}
