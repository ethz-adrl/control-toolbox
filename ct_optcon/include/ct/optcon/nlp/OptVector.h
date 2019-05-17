/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#pragma once

namespace ct {
namespace optcon {
namespace tpl {


/**
 * @ingroup    NLP
 *
 * @brief      Class containing and managing all the optimization variables used
 *             for in the NLP solver IPOPT and SNOPT.
 */
template <typename SCALAR>
class OptVector
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using VectorXs = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
    using VectorXi = Eigen::Matrix<int, Eigen::Dynamic, 1>;
    using MapVecXs = Eigen::Map<VectorXs>;
    using MapVecXi = Eigen::Map<VectorXi>;
    using MapConstVecXs = Eigen::Map<const VectorXs>;

    OptVector() = delete;

    /**
     * @brief      { Constructor resizing the vectors of the optimization
     *             variables to the correct size }
     *
     * @param[in]  n     The number of the optimization variables
     */
    OptVector(const size_t n) : updateCount_(0)
    {
        x_.resize(n);
        x_.setZero();
        xInit_.resize(n);
        xLb_.resize(n);
        xUb_.resize(n);
        xLb_.setConstant(std::numeric_limits<SCALAR>::lowest());
        xUb_.setConstant(std::numeric_limits<SCALAR>::max());
        xMul_.resize(n);
        xMul_.setZero();
        xState_.resize(n);
        xState_.setZero();
        zUpper_.resize(n);
        zUpper_.setZero();
        zLow_.resize(n);
        zLow_.setZero();
    }

    /**
     * @brief      Resizes the vectors of the constraint variables to the
     *             correct size
     *
     * @param[in]  m     The number of constraints
     */
    void resizeConstraintVars(size_t m)
    {
        zMul_.resize(m + 1);
        zMul_.setZero();
        zState_.resize(m + 1);
        zState_.setZero();
    }

    void reset() { updateCount_ = 0; }
    /**
     * @brief      Destructor
     */
    virtual ~OptVector() = default;
    /**
     * @brief      Resizes the vectors of the optimization variables
     *
     * @param[in]  size  The size of the new optimization variables
     */
    void resizeOptimizationVars(const size_t size)
    {
        x_.resize(size);
        xInit_.resize(size);
        xLb_.resize(size);
        xUb_.resize(size);
        lambda_.resize(size);
        zUpper_.resize(size);
        zLow_.resize(size);
    }


    /**
     * @brief      Resets the optimization variables
     */
    void setZero()
    {
        x_.setZero();
        xInit_.setZero();
        lambda_.setZero();
        zUpper_.setZero();
        zLow_.setZero();
    }

    void setRandomInitialGuess()
    {
        x_.setRandom();
        xInit_.setRandom();
    }

    void setInitialGuess(const VectorXs& xinit)
    {
        x_ = xinit;
        xInit_ = xinit;
    }

    /**
     * @brief      Checks if the optimization variables have to correct size
     *
     * @param[in]  n     The number of optimization variables
     *
     * @return     returns true of the dimensions match
     */
    bool checkOptimizationVarDimension(const unsigned int n)
    {
        bool xDim = x_.size() == n ? true : false;
        bool xLDim = xLb_.size() == n ? true : false;
        bool xUDim = xUb_.size() == n ? true : false;
        return xDim && xLDim && xUDim;
    }

    /**
     * @brief      Sets the optimization variable bounds.
     *
     * @param[in]  xLb   The lower optimization variable bound
     * @param[in]  xUb   The upper optimization variable bound
     */
    void setBounds(const VectorXs& xLb, const VectorXs& xUb)
    {
        xLb_ = xLb;
        xUb_ = xUb;
    }


    /**
     * @brief      Gets the lower bounds of the optimization variables.
     *
     * @param[out] x     Lower bound
     */
    void getLowerBounds(MapVecXs& x) const { x = xLb_; }
    /**
     * @brief      Gets the upper bounds of the optimization variables.
     *
     * @param[out]      x     The upper bound
     */
    void getUpperBounds(MapVecXs& x) const { x = xUb_; }
    /**
     * @brief      Return the state and the multiplier of the optimization
     *             variables, used in the NLP solver SNOPT.
     *
     * @param[in]  n       { The number of optimization variables }
     * @param[out] xMul    The optimization variables multiplier
     * @param[out] xState  The optimization variables state
     */
    void getOptimizationMultState(const size_t n, MapVecXs& xMul, MapVecXi& xState) const
    {
        assert(n == xMul_.size());
        assert(n == xState_.size());
        xMul = xMul_;
        xState = xState_;
    }

    /**
     * @brief      Gets the constraint multiplier and state, used in the NLP
     *             solver SNOPT.
     *
     * @param[in]  m       { The number of constraints }
     * @param[out] zMul    The constraint variable multiplier
     * @param[out] zState  The constraint variable state
     */
    void getConstraintsMultState(const size_t m, MapVecXs& zMul, MapVecXi& zState) const
    {
        assert(m == zMul_.size());
        assert(m == zState_.size());
        zMul = zMul_;
        zState = zState_;
    }

    /**
     * @brief      Returns the number of optimization variables
     *
     * @return     the number of optimization variables
     */
    size_t size() const { return x_.size(); }
    /**
     * @brief      Gets the bound multipliers used in the NLP solver IPOPT.
     *
     * @param[in]  n     { The number of optimization variables }
     * @param[out] low   The value for the lower bound multiplier
     * @param[out] up    The value for the upper bound multiplier
     */
    void getBoundMultipliers(size_t n, MapVecXs& low, MapVecXs& up) const
    {
        assert(n == static_cast<size_t>(zLow_.size()));
        low = zLow_;
        up = zUpper_;
    }

    /**
     * @brief      Gets the values of the constraint multipliers.
     *
     * @param[in]  m     { The number of constraints }
     * @param[out] x     The values of the constraint multipliers
     */
    void getLambdaVars(size_t m, MapVecXs& x) const
    {
        assert(m == static_cast<size_t>(lambda_.size()));
        x = lambda_;
    }

    /**
     * @brief      Gets the optimization variables.
     *
     * @param[in]  n     { The number of optimization variables }
     * @param[out]      x     The optimization variables
     */
    void getOptimizationVars(size_t n, MapVecXs& x) const
    {
        assert(n == x_.size());
        x = x_;
    }

    const VectorXs& getOptimizationVars() const { return x_; }
    void getInitialGuess(size_t n, MapVecXs& x) const
    {
        assert(n == static_cast<size_t>(xInit_.size()));
        x = xInit_;
    }

    /**
     * @brief      Extracts the solution from ipopt and stores them into class
     *             variables
     *
     * @param[in]  x       The optimization variables
     * @param[in]  zL      The lower bound multiplier
     * @param[in]  zU      The upper bound multiplier
     * @param[in]  lambda  The constraint multiplier
     */
    void setNewIpoptSolution(const MapConstVecXs& x,
        const MapConstVecXs& zL,
        const MapConstVecXs& zU,
        const MapConstVecXs& lambda)
    {
        x_ = x;
        zLow_ = zL;
        zUpper_ = zU;
        lambda_ = lambda;
    }

    /**
     * @brief      Extracts the solution from snopt and stores it into class
     *             variables
     *
     * @param[in]  x       The optimization variables
     * @param[in]  xMul    The optimization variables multiplier
     * @param[in]  xState  The optimization variables state
     * @param[in]  fMul    The constraints multiplier
     * @param[in]  fState  The constraints state
     */
    void setNewSnoptSolution(const MapVecXs& x,
        const MapVecXs& xMul,
        const MapVecXi& xState,
        const MapVecXs& fMul,
        const MapVecXi& fState)
    {
        x_ = x;
        xMul_ = xMul;
        xState_ = xState;
        zMul_ = fMul;
        zState_ = fState;
    }

    /**
     * @brief      Sets the updates optimization variables from the NLP solver
     *             and updates the counter
     *
     * @param[in]  x     The updates primal variables
     */
    void setOptimizationVars(const MapConstVecXs& x)
    {
        x_ = x;
        updateCount_++;
    }

    void setOptimizationVars(const VectorXs& x)
    {
        x_ = x;
        updateCount_++;
    }


    /**
     * @brief      Returns the update counter
     *
     * @return     The update counter
     */
    size_t getUpdateCount() const { return updateCount_; }
protected:
    VectorXs x_; /*!< The optimization variables */
    VectorXs xInit_;
    VectorXs xLb_; /*!< lower bound on optimization vector */
    VectorXs xUb_; /*!< upper bound on optimization vector */

    VectorXs zUpper_; /*!< The upper bound multiplier, used in IPOPT */
    VectorXs zLow_;   /*!< The lower bound multiplier, used in IPOPT */
    VectorXs lambda_; /*!< The constraint multiplier, used in IPOPT */

    // Snopt variables
    VectorXs xMul_;   /*!< The optimization variable multiplier, used in SNOPT */
    VectorXi xState_; /*!< The optimization variable state, used in SNOPT */
    VectorXs zMul_;   /*!< The constraint multiplier, used in SNOPT */
    VectorXi zState_; /*!< The constraint state, used in SNOPT */

    size_t updateCount_; /*!< The number of optimization variable updates */
};
}

using OptVector = tpl::OptVector<double>;
}
}
