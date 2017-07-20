#include <ct/core/core.h>
#include "system/TestNonlinearSystem.h"

using namespace ct::core;



// template<typename SCALAR>
// Eigen::Matrix<SCALAR, state_dim, 1> integrate(Eigen::Matrix<SCALAR, state_dim, 1>& x0)
// {
//     double w_n = 100;
//     std::shared_ptr<TestNonlinearSystemAD> nonlinearSystemAD(new tpl::TestNonlinearSystem<AD_Scalar>(AD_Scalar(w_n)));
//     IntegratorRK4<state_dim> integrator()
// }
// 

class IntegrationWrapperAd
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // typedefs for the auto-differentiable system
    typedef tpl::TestNonlinearSystem<ADCGScalar> TestNonlinearSystemAD;
    const static size_t state_dim = TestNonlinearSystem::STATE_DIM;
    const static size_t control_dim = TestNonlinearSystem::CONTROL_DIM;

    typedef IntegratorEuler<state_dim, ADCGScalar> integrator_type;
    typedef std::shared_ptr<integrator_type> integrator_type_ptr;

    IntegrationWrapperAd(size_t w_n)
    {
        nonlinearSystemAD_ = std::shared_ptr<TestNonlinearSystemAD>(new TestNonlinearSystemAD(ADCGScalar(w_n)));

        integrator_ = integrator_type_ptr(new integrator_type(nonlinearSystemAD_));
    }

    Eigen::Matrix<ADCGScalar, state_dim, 1> integrate()
    {

    }



private:
    std::shared_ptr<TestNonlinearSystemAD> nonlinearSystemAD_;
    integrator_type_ptr integrator_;


};



int main()
{
    double w_n = 100;
    IntegrationWrapperAd wrapper(w_n);


    // std::shared_ptr<TestNonlinearSystemAD> nonlinearSystemAD(new tpl::TestNonlinearSystem<AD_Scalar>(AD_Scalar(w_n)));



    std::cout << "Hello world" << std::endl;
    return 0;
}