/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/robot/jacobian/JacobianBase.h>

namespace ct {
namespace rbd {
namespace tpl {

/**
 * \ingroup OS
 * @Brief This class provides extra functionalities for performing the operational space Jacobain matrix algebra.
 * such as calculating the Moore-Penrose pseudo-inverse and Null space projectors.
 */
template <size_t NUM_OUTPUTS, size_t NUM_JOINTS, typename SCALAR>
class OperationalJacobianBase : public JacobianBase<NUM_OUTPUTS, NUM_JOINTS, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<OperationalJacobianBase<NUM_OUTPUTS, NUM_JOINTS, SCALAR>> ptr;
    typedef typename JacobianBase<NUM_OUTPUTS, NUM_JOINTS, SCALAR>::state_t state_t;
    typedef typename JacobianBase<NUM_OUTPUTS, NUM_JOINTS, SCALAR>::jacobian_t jacobian_t;
    typedef typename JacobianBase<NUM_OUTPUTS, NUM_JOINTS, SCALAR>::jacobian_inv_t jacobian_inv_t;
    typedef Eigen::Matrix<SCALAR, 6 + NUM_JOINTS, 6 + NUM_JOINTS> square_matrix_t;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;


    OperationalJacobianBase() { resetUpdatedFlags(); }
    virtual ~OperationalJacobianBase() {}
    /**
	 * This function gets the RBD state and the the user defined weighting matrix in order to calculate the
	 * the floating-base Jacobian, its right pseudo-inverse, and null space and their corresponding time
	 * derivatives.
	 * @param state     State of the RBD
	 * @param weighting The weighting function for the calculating the  pseudo-inverse and null space
	 */
    void updateState(const state_t& state, const square_matrix_t& weighting = square_matrix_t::Identity())
    {
        resetUpdatedFlags();
        state_ = state;
        W_ = weighting;
    }

    //!  This method gets the floating-base Jacobian
    const jacobian_t& J()
    {
        if (JUpdated_ == true)
            return J_;

        JUpdated_ = true;
        getJacobianOrigin(state_, J_);
        return J_;
    }

    //!  This method gets the time derivative of the floating-base Jacobian
    const jacobian_t& dJdt()
    {
        if (dJdtUpdated_ == true)
            return dJdt_;

        dJdtUpdated_ = true;
        getJacobianOriginDerivative(state_, dJdt_);
        return dJdt_;
    }

    //!  This method calculates the right inverse using W as the weighting matrix
    const jacobian_inv_t& JdagerLDLT()
    {
        if (JdagerUpdated_ == true)
            return Jdager_;

        JdagerUpdated_ = true;
        Jdager_ = W_ * J().transpose() *
                  (J() * W_ * J().transpose()).ldlt().solve(MatrixXs::Identity(NUM_OUTPUTS, NUM_OUTPUTS));
        return Jdager_;
    }

    const jacobian_inv_t& JdagerSVD()
    {
        if (JdagerUpdated_ == true)
            return Jdager_;

        JdagerUpdated_ = true;
        Eigen::JacobiSVD<MatrixXs> svd(J(), Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> sing_values(
            svd.matrixV().cols(), 1);  // size of E has same size as columns of V
        sing_values = (svd.singularValues().array() > 1e-9).select(svd.singularValues().array().inverse(), 0);
        Jdager_ = svd.matrixV() * sing_values.asDiagonal() * svd.matrixU().transpose();
        return Jdager_;
    }

    //!  This method calculates the time derivative of right inverse
    const jacobian_inv_t& dJdagerdt()
    {
        // if(dJdagerdtUpdated_==true)  return dJdagerdt_;

        dJdagerdtUpdated_ = true;
        dJdagerdt_ = -JdagerLDLT() * dJdt() * JdagerLDLT();
        return dJdagerdt_;
    }

    //!  This method calculates the null space projector
    const square_matrix_t& P()
    {
        if (PUpdated_ == true)
            return P_;

        PUpdated_ = true;
        P_ = square_matrix_t::Identity() - JdagerLDLT() * J();
        return P_;
    }

    //!  This method calculates the time derivative of the null space projector
    const square_matrix_t& dPdt()
    {
        if (dPdtUpdated_ == true)
            return dPdt_;

        dPdtUpdated_ = true;
        dPdt_ = -JdagerLDLT() * dJdt() * P();
        return dPdt_;
    }

    /**
	 * A method for reseting the user defined flags. This function is called is the new state is set
	 * using the update() method
	 */
    virtual void resetUserUpdatedFlags() {}
    /**
	 * This methods calculates the Jacobian of the floating-base Jacobian in the Origin (World or Inertia)
	 * frame.
	 * @param state State of the RBD
	 * @param J     floating-base Jacobian in the Origin frame
	 */
    virtual void getJacobianOrigin(const state_t& state, jacobian_t& J) = 0;

    /**
	 * This methods calculates the time derivative of the Jacobian of the floating-base function in the
	 * Origin (World or Inertia) frame.
	 * @param state State of the RBD
	 * @param dJdt  Time derivative of the floating-base Jacobian in the Origin frame
	 */
    virtual void getJacobianOriginDerivative(const state_t& state, jacobian_t& dJdt) = 0;


protected:
    jacobian_t J_;

private:
    virtual void resetUpdatedFlags()
    {
        JUpdated_ = false;
        dJdtUpdated_ = false;
        JdagerUpdated_ = false;
        dJdagerdtUpdated_ = false;
        PUpdated_ = false;
        dPdtUpdated_ = false;
        resetUserUpdatedFlags();
    }

    /*
	 * variables
	 */
    bool JUpdated_;
    bool dJdtUpdated_;
    bool JdagerUpdated_;
    bool dJdagerdtUpdated_;
    bool PUpdated_;
    bool dPdtUpdated_;

    state_t state_;
    square_matrix_t W_;

    jacobian_t dJdt_;
    jacobian_inv_t Jdager_;
    jacobian_inv_t dJdagerdt_;
    square_matrix_t P_;
    square_matrix_t dPdt_;
};

}  // namespace tpl

template <size_t NUM_OUTPUTS, size_t NUM_JOINTS>
using OperationalJacobianBase = tpl::OperationalJacobianBase<NUM_OUTPUTS, NUM_JOINTS, double>;

}  // namespace rbd
}  // namespace ct
