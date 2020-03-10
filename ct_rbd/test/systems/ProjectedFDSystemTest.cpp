/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-value"
#pragma GCC diagnostic ignored "-Wunused-variable"



#include <gtest/gtest.h>

#include <ct/rbd/rbd.h>
#include "../models/testhyq/RobCoGenTestHyQ.h"

using namespace ct;
using namespace ct::rbd;

TEST(ProjectedFDSystemTest, projected_forward_dynamics_test)
{
    constexpr bool VERBOSE = false;
    constexpr size_t nTests = 10;
    constexpr size_t nIntegrationSteps = 50;
    constexpr double dt = 0.001;

    const size_t STATE_DIM_QUAT = ProjectedFDSystem<TestHyQ::Dynamics, true>::STATE_DIM;
    const size_t STATE_DIM_EULER = ProjectedFDSystem<TestHyQ::Dynamics, false>::STATE_DIM;

    std::shared_ptr<ProjectedFDSystem<TestHyQ::Dynamics, true>> dynamicsQuat(
        new ProjectedFDSystem<TestHyQ::Dynamics, true>);
    std::shared_ptr<ProjectedFDSystem<TestHyQ::Dynamics, false>> dynamicsEuler(
        new ProjectedFDSystem<TestHyQ::Dynamics, false>);

    core::Integrator<STATE_DIM_QUAT> integratorQuat(dynamicsQuat, ct::core::RK4);
    core::Integrator<STATE_DIM_EULER> integratorEuler(dynamicsEuler, ct::core::RK4);

    TestHyQ::Kinematics kinematics;
    RBDState<12> state, finalQuat, finalEuler;
    TestHyQ::Dynamics::EE_in_contact_t contactFlags;
    Eigen::Matrix<bool, 4, 1> eeContactConfig;
    std::vector<kindr::Velocity<double, 3>> feetVelocities;

    for (size_t i = 0; i < nTests; i++)
    {
        // Create initial conditions
        state.setRandom();
        eeContactConfig.setRandom();
        for (size_t j = 0; j < 4; j++)
        {
            contactFlags[j] = eeContactConfig[j];
        }

        // Set initial conditions
        dynamicsQuat->setEEInContact(contactFlags);
        dynamicsEuler->setEEInContact(contactFlags);
        core::StateVector<STATE_DIM_QUAT> stateQuat = state.toStateVectorQuaternion();
        core::StateVector<STATE_DIM_EULER> stateEuler = state.toStateVectorEulerXyz();
        feetVelocities.clear();
        for (size_t k = 0; k < 4; k++)
        {
            feetVelocities.push_back(kinematics.getEEVelocityInWorld(k, state));
        }

        for (size_t j = 0; j < nIntegrationSteps; j++)
        {
            integratorQuat.integrate_n_steps(stateQuat, 0, 1, dt);
            integratorEuler.integrate_n_steps(stateEuler, 0, 1, dt);

            finalQuat.fromStateVectorQuaternion(stateQuat);
            finalEuler.fromStateVectorEulerXyz(stateEuler);

            if (VERBOSE)
            {
                std::cout << "test " << i << ", integration: " << j << std::endl;
                std::cout << "diff: "
                          << finalQuat.toStateVectorEulerXyz().transpose() -
                                 finalEuler.toStateVectorEulerXyz().transpose()
                          << std::endl;
            }

            // check if both states are the same
            ASSERT_TRUE(finalQuat.isApprox(finalEuler, 1e-6));

            // check that EE in contact do not move
            for (size_t k = 0; k < 4; k++)
            {
                if (contactFlags[k])
                {
                    if (VERBOSE)
                    {
                        std::cout << "test " << i << ", integration: " << j << ", foothold "
                                  << eeContactConfig.transpose() << ", foot " << k << std::endl;
                        std::cout << "Quat diff ||v||"
                                  << (feetVelocities[k] - kinematics.getEEVelocityInWorld(k, finalQuat))
                                         .toImplementation()
                                         .transpose()
                                  << std::endl;
                        std::cout << "Euler diff ||v||"
                                  << (feetVelocities[k] - kinematics.getEEVelocityInWorld(k, finalEuler))
                                         .toImplementation()
                                         .transpose()
                                  << std::endl;
                    }

                    ASSERT_TRUE(kinematics.getEEVelocityInWorld(k, finalQuat)
                                    .toImplementation()
                                    .isApprox(feetVelocities[k].toImplementation(), 1e-6));
                    ASSERT_TRUE(kinematics.getEEVelocityInWorld(k, finalEuler)
                                    .toImplementation()
                                    .isApprox(feetVelocities[k].toImplementation(), 1e-6));
                }
            }
        }
    }
}


TEST(ProjectedFDSystemTest, projected_forward_dynamics_autodiff_test)
{
    constexpr bool VERBOSE = false;
    constexpr int NTESTS = 10;

    constexpr size_t state_dim = ProjectedFDSystem<TestHyQ::Dynamics, false>::STATE_DIM;
    constexpr size_t control_dim = ProjectedFDSystem<TestHyQ::Dynamics, false>::CONTROL_DIM;

    typedef CppAD::AD<double> AD_Scalar;
    using ProjectedFDSystem_t = ProjectedFDSystem<TestHyQ::Dynamics, false>;
    using ProjectedFDSystemAD_t = ProjectedFDSystem<TestHyQ::tpl::Dynamics<AD_Scalar>, false>;

    std::shared_ptr<ProjectedFDSystem_t> hyq(new ProjectedFDSystem_t);
    std::shared_ptr<ProjectedFDSystemAD_t> hyqAD(new ProjectedFDSystemAD_t);

    core::SystemLinearizer<state_dim, control_dim> sysLinearizer(hyq);
    core::AutoDiffLinearizer<state_dim, control_dim> adLinearizer(hyqAD);

    // create state, control and time variables
    RBDState<ProjectedFDSystem<TestHyQ::Dynamics, false>::Dynamics::NJOINTS> state;
    core::StateVector<state_dim> x;
    core::ControlVector<control_dim> u;
    double t = 0;
    typedef Eigen::Matrix<double, state_dim, state_dim> A_type;
    typedef Eigen::Matrix<double, state_dim, control_dim> B_type;

    for (int i = 0; i < NTESTS; i++)
    {
        // Set linearization point
        state.setRandom();
        x = state.toStateVectorEulerXyz();
        u.setRandom();

        // use the numerical differentiation linearizer
        A_type A_system = sysLinearizer.getDerivativeState(x, u, t);
        B_type B_system = sysLinearizer.getDerivativeControl(x, u, t);

        // use the auto differentiation linearizer
        A_type A_ad = adLinearizer.getDerivativeState(x, u, t);
        B_type B_ad = adLinearizer.getDerivativeControl(x, u, t);

        if (VERBOSE)
        {
            std::cout << "test " << i << std::endl
                      << "A_ad:\n"
                      << A_ad << std::endl
                      << "A_sys:\n"
                      << A_system << std::endl
                      << "A_sys - A_ad:\n"
                      << A_system - A_ad << std::endl
                      << "B_sys:\n"
                      << B_system << std::endl
                      << "B_ad:\n"
                      << B_ad << std::endl
                      << "B_sys - B_ad:\n"
                      << B_system - B_ad << std::endl;
        }

        ASSERT_LT((A_system - A_ad).array().abs().maxCoeff(), 1e-3);
        ASSERT_LT((B_system - B_ad).array().abs().maxCoeff(), 1e-3);
    }
}

TEST(ProjectedFDSystemTest, projected_forward_dynamics_codegen_test)
{
#ifdef NDEBUG
    constexpr size_t state_dim = ProjectedFDSystem<TestHyQ::Dynamics, false>::STATE_DIM;
    constexpr size_t control_dim = ProjectedFDSystem<TestHyQ::Dynamics, false>::CONTROL_DIM;

    typedef core::ADCodegenLinearizer<state_dim, control_dim>::ADCGScalar ADCGScalar;
    using ProjectedFDSystemADCG_t = ProjectedFDSystem<TestHyQ::tpl::Dynamics<ADCGScalar>, false>;

    std::shared_ptr<ProjectedFDSystemADCG_t> hyqADCG(new ProjectedFDSystemADCG_t);
    core::ADCodegenLinearizer<state_dim, control_dim> adcgLinearizer(hyqADCG);

    try
    {
        std::cout << "Testing codegen..." << std::endl;
        adcgLinearizer.generateCode("ProjectedFDSystemTest_hyqADCG_JITLib");
        std::cout << "... done!" << std::endl;
    } catch (std::exception& e)
    {
        std::cout << "Exception thrown: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
#else
    std::cout << "Compile with -DNDEBUG to enable codegen" << std::endl;
#endif
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


#pragma GCC diagnostic pop

