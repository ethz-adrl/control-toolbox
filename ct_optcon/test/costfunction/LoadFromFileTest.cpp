/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <gtest/gtest.h>

#include <ct/optcon/optcon.h>

#include "compareCostFunctions.h"
#include "costfunction_test_dir.h"

std::string costFunctionFile = std::string(COSTFUNCTION_TEST_DIR) + "/example_cost_file.info";
const bool verbose = false;

const size_t state_dim = 12;
const size_t control_dim = 6;

// test state and control
ct::core::StateVector<state_dim> x = ct::core::StateVector<state_dim>::Random();
ct::core::ControlVector<control_dim> u = ct::core::ControlVector<control_dim>::Random();
double t = 0.0;

namespace ct {
namespace optcon {
namespace example {


//! Test case: trying to load analytical cost function directly from file
TEST(LoadCostFromFileTest, LoadAnalyticalDirect)
{
    try
    {
        std::shared_ptr<ct::optcon::CostFunctionAnalytical<state_dim, control_dim>> costFun(
            new ct::optcon::CostFunctionAnalytical<state_dim, control_dim>(costFunctionFile, verbose));

        // check cloning
        std::shared_ptr<ct::optcon::CostFunctionAnalytical<state_dim, control_dim>> costFun_cloned(costFun->clone());

        // set states and compare costs and gradients
        costFun->setCurrentStateAndControl(x, u, t);
        costFun_cloned->setCurrentStateAndControl(x, u, t);
        compareCostFunctionOutput<state_dim, control_dim>(*costFun, *costFun_cloned);

    } catch (std::runtime_error& e)
    {
        std::cout << "Exception caught: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}


//! Test case: trying to load analytical cost terms separately and then add them cost function
TEST(LoadCostFromFileTest, LoadAnalyticalViaTerms)
{
    try
    {
        std::shared_ptr<ct::optcon::CostFunctionAnalytical<state_dim, control_dim>> costFun(
            new ct::optcon::CostFunctionAnalytical<state_dim, control_dim>());

        // here, we take the detour via loading the terms separately.
        std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> termQuadratic_intermediate(
            new ct::optcon::TermQuadratic<state_dim, control_dim>);
        std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> termQuadratic_final(
            new ct::optcon::TermQuadratic<state_dim, control_dim>);

        termQuadratic_intermediate->loadConfigFile(costFunctionFile, "term0", verbose);
        termQuadratic_final->loadConfigFile(costFunctionFile, "term1", verbose);

        costFun->addIntermediateTerm(termQuadratic_intermediate);
        costFun->addFinalTerm(termQuadratic_final);

        std::shared_ptr<ct::optcon::CostFunctionAnalytical<state_dim, control_dim>> costFun_cloned(costFun->clone());


        // set states and compare costs and gradients
        costFun->setCurrentStateAndControl(x, u, t);
        costFun_cloned->setCurrentStateAndControl(x, u, t);
        compareCostFunctionOutput<state_dim, control_dim>(*costFun, *costFun_cloned);

    } catch (std::runtime_error& e)
    {
        std::cout << "Exception caught: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}


//! Test case: trying to load AD cost terms separately and then add them to the cost function
TEST(LoadCostFromFileTest, LoadADViaTerms)
{
    try
    {
        std::shared_ptr<ct::optcon::CostFunctionAD<state_dim, control_dim>> costFun(
            new ct::optcon::CostFunctionAD<state_dim, control_dim>());

        // here, we take the detour via loading the terms separately.
        std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim, double, ct::core::ADCGScalar>>
        termQuadraticAD_intermediate(
            new ct::optcon::TermQuadratic<state_dim, control_dim, double, ct::core::ADCGScalar>);
        std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim, double, ct::core::ADCGScalar>>
        termQuadraticAD_final(new ct::optcon::TermQuadratic<state_dim, control_dim, double, ct::core::ADCGScalar>);

        termQuadraticAD_intermediate->loadConfigFile(costFunctionFile, "term0", verbose);
        termQuadraticAD_final->loadConfigFile(costFunctionFile, "term1", verbose);

        costFun->addIntermediateADTerm(termQuadraticAD_intermediate);
        costFun->addFinalADTerm(termQuadraticAD_final);
        costFun->initialize();  // todo we need to get rid of this initialize call()

        std::shared_ptr<ct::optcon::CostFunctionAD<state_dim, control_dim>> costFun_cloned(costFun->clone());

        // set states and compare costs and gradients
        costFun->setCurrentStateAndControl(x, u, t);
        costFun_cloned->setCurrentStateAndControl(x, u, t);
        compareCostFunctionOutput<state_dim, control_dim>(*costFun, *costFun_cloned);

    } catch (std::runtime_error& e)
    {
        std::cout << "Exception caught: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}

//! Test case: trying to load AD cost directly, but calling standard constructor first.
TEST(LoadCostFromFileTest, LoadADDirect1)
{
    try
    {
        // first create object
        std::shared_ptr<ct::optcon::CostFunctionAD<state_dim, control_dim>> costFun(
            new ct::optcon::CostFunctionAD<state_dim, control_dim>());
        // then load and initialize
        costFun->loadFromConfigFile(costFunctionFile, verbose);
        costFun->initialize();  // todo need to get rid of this init call

        std::shared_ptr<ct::optcon::CostFunctionAD<state_dim, control_dim>> costFun_cloned(costFun->clone());

        // set states and compare costs and gradients
        costFun->setCurrentStateAndControl(x, u, t);
        costFun_cloned->setCurrentStateAndControl(x, u, t);
        compareCostFunctionOutput<state_dim, control_dim>(*costFun, *costFun_cloned);

    } catch (std::runtime_error& e)
    {
        std::cout << "Exception caught: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}


//! Test case: trying to load AD cost directly
TEST(LoadCostFromFileTest, LoadADDirect2)
{
    try
    {
        // test constructor taking the cost function file directly
        std::shared_ptr<ct::optcon::CostFunctionAD<state_dim, control_dim>> costFun(
            new ct::optcon::CostFunctionAD<state_dim, control_dim>(costFunctionFile, verbose));
        costFun->initialize();  // todo need to get rid of this init call

        std::shared_ptr<ct::optcon::CostFunctionAD<state_dim, control_dim>> costFun_cloned(costFun->clone());

        // set states and compare costs and gradients
        costFun->setCurrentStateAndControl(x, u, t);
        costFun_cloned->setCurrentStateAndControl(x, u, t);
        compareCostFunctionOutput<state_dim, control_dim>(*costFun, *costFun_cloned);

    } catch (std::runtime_error& e)
    {
        std::cout << "Exception caught: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}


}  // namespace example
}  // namespace optcon
}  // namespace ct


int main(int argc, char** argv)
{
    using namespace ct::optcon::example;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
