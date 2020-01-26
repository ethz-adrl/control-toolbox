/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * In this file, derivative code including the actuator dynamics is generated.
 */

#include <ct/rbd/rbd.h>

#include <cmath>
#include <memory>

#include <ct/models/InvertedPendulum/InvertedPendulum.h>
#include <ct/rbd/systems/FixBaseFDSystem.h>

#include <ct/models/CodegenOutputDirs.h>

const double K_SPRING = 180.9;  //! uniform spring constant
const double R_GEAR = 50.0;     //! constant gear ratio

const size_t njoints = ct::rbd::InvertedPendulum::Kinematics::NJOINTS;
const size_t actuator_state_dim = 1;
const size_t state_dim_full = 2 * njoints + actuator_state_dim;

using Scalar = ct::core::ADCodegenLinearizer<state_dim_full, njoints>::ADCGScalar;
using SEADynamicsFirstOrderAD = ct::rbd::SEADynamicsFirstOrder<njoints, Scalar>;
using InvertedPendulumDynamicsAD =  ct::rbd::InvertedPendulum::tpl::Dynamics<Scalar>;
using InvertedPendulumSystemAD = ct::rbd::FixBaseFDSystem<InvertedPendulumDynamicsAD, actuator_state_dim, false>;

template <typename SCALAR>
using control_vector_t = typename ct::rbd::InvertedPendulum::tpl::Dynamics<SCALAR>::control_vector_t;

template <typename SCALAR>
using ExtLinkForces_t = typename ct::rbd::InvertedPendulum::tpl::Dynamics<SCALAR>::ExtLinkForces_t;


int main(int argc, char** argv)
{
    std::shared_ptr<SEADynamicsFirstOrderAD> actuatorDynamicsAD(new SEADynamicsFirstOrderAD(K_SPRING, R_GEAR));
    std::shared_ptr<InvertedPendulumSystemAD> ip (new InvertedPendulumSystemAD(actuatorDynamicsAD));
    ct::core::ADCodegenLinearizer<state_dim_full, njoints> adLinearizer(ip);

    try
    {
        std::cout << "generating using forward mode" << std::endl;
        adLinearizer.generateCode("InvertedPendulumActDynLinearizedForward", ct::models::IP_CODEGEN_OUTPUT_DIR,
            ct::core::CODEGEN_TEMPLATE_DIR, "models", "InvertedPendulum", false);

        std::cout << "... done!" << std::endl;


    } catch (const std::runtime_error& e)
    {
        std::cout << "code generation failed: " << e.what() << std::endl;
    }

    return 0;
}
