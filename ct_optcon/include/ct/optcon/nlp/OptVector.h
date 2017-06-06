/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/


#ifndef CT_OPTCON_NLP_OPTVECTOR_H_
#define CT_OPTCON_NLP_OPTVECTOR_H_

namespace ct {
namespace optcon {

/**
 * @ingroup    NLP
 *
 * @brief      Class containing and managing all the optimization variables used
 *             for in the NLP solver IPOPT and SNOPT.
 */
class OptVector
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef double Number;
	typedef Eigen::Matrix<Number, Eigen::Dynamic, 1> VectorXd;
	typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VectorXi;
	typedef Eigen::Map<VectorXd> MapVecXd;
	typedef Eigen::Map<VectorXi> MapVecXi;
	typedef Eigen::Map<const VectorXd> MapConstVecXd;

	OptVector() = delete;

	/**
	 * @brief      { Constructor resizing the vectors of the optimization
	 *             variables to the correct size }
	 *
	 * @param[in]  n     The number of the optimization variables
	 */
	OptVector(const size_t n) :
	updateCount_(0)
	{
		x_.resize(n);
		x_.setZero();
		xLb_.resize(n);
		xUb_.resize(n);
		xLb_.setConstant(std::numeric_limits<double>::lowest());
		xUb_.setConstant(std::numeric_limits<double>::max());
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
		zMul_.resize(m+1);
		zMul_.setZero();
		zState_.resize(m+1);
		zState_.setZero();
	}

	/**
	 * @brief      Destructor
	 */
	virtual ~OptVector(){}

	 
	/**
	 * @brief      Resizes the vectors of the optimization variables
	 *
	 * @param[in]  size  The size of the new optimization variables
	 */
	void resizeOptimizationVars(const size_t size) {
		x_.resize(size);
		xLb_.resize(size);
		xUb_.resize(size);
		lambda_.resize(size);
		zUpper_.resize(size);
		zLow_.resize(size);
	}


	/**
	 * @brief      Resets the optimization variables
	 */
	void setZero() {
		x_.setZero();
		xLb_.setZero();
		xUb_.setZero();
		lambda_.setZero();
		zUpper_.setZero();
		zLow_.setZero();
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
	void setBounds(const Eigen::VectorXd& xLb, const Eigen::VectorXd& xUb){
		xLb_ = xLb;
		xUb_ = xUb;
	}


	/**
	 * @brief      Gets the lower bounds of the optimization variables.
	 *
	 * @param[out] x     Lower bound
	 */
	void getLowerBounds(MapVecXd& x) const {
		x = xLb_;
	}

	/**
	 * @brief      Gets the upper bounds of the optimization variables.
	 *
	 * @param[out]      x     The upper bound
	 */
	void getUpperBounds(MapVecXd& x) const {
		x = xUb_;
	}

	/**
	 * @brief      Return the state and the multiplier of the optimization
	 *             variables, used in the NLP solver SNOPT.
	 *
	 * @param[in]  n       { The number of optimization variables }
	 * @param[out] xMul    The optimization variables multiplier
	 * @param[out] xState  The optimization variables state
	 */
	void getOptimizationMultState(const size_t n, MapVecXd& xMul, Eigen::Map<Eigen::VectorXi>& xState) const {
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
	void getConstraintsMultState(const size_t m, MapVecXd& zMul, Eigen::Map<Eigen::VectorXi>& zState) const {
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
	size_t size() const {return x_.size();}

	/**
	 * @brief      Gets the bound multipliers used in the NLP solver IPOPT.
	 *
	 * @param[in]  n     { The number of optimization variables }
	 * @param[out] low   The value for the lower bound multiplier
	 * @param[out] up    The value for the upper bound multiplier
	 */
	void getBoundMultipliers(size_t n, MapVecXd& low, MapVecXd& up) const {
		assert(n == zLow_.size());
		low = zLow_;
		up = zUpper_;
	}
	
	/**
	 * @brief      Gets the values of the constraint multipliers.
	 *
	 * @param[in]  m     { The number of constraints }
	 * @param[out] x     The values of the constraint multipliers
	 */
	void getLambdaVars(size_t m, MapVecXd& x) const {
		assert(m == lambda_.size());
		x = lambda_;
	}

	/**
	 * @brief      Gets the optimization variables.
	 *
	 * @param[in]  n     { The number of optimization variables }
	 * @param[out]      x     The optimization variables
	 */
	void getOptimizationVars(size_t n, MapVecXd& x)  const {
		assert (n == x_.size());
		x = x_;
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
	void setNewIpoptSolution(
		const MapConstVecXd& x, 
		const MapConstVecXd& zL, 
		const MapConstVecXd& zU, 
		const MapConstVecXd& lambda)
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
	void setNewSnoptSolution(const MapVecXd& x, const MapVecXd& xMul, 
		const MapVecXi& xState, const MapVecXd& fMul, const MapVecXi& fState)
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
	void setOptimizationVars(const MapConstVecXd& x)
	{
		x_ = x;
		updateCount_++;
	}

	// Eigen::VectorXd getPrimalVars() const{
	// 	return x_;
	// }

	/**
	 * @brief      Returns the update counter
	 *
	 * @return     The update counter
	 */
	size_t getUpdateCount() const{
		return updateCount_;
	}


protected:
	Eigen::VectorXd x_; 	/*!< The optimization variables */
	Eigen::VectorXd xLb_;	/*!< lower bound on optimization vector */
	Eigen::VectorXd xUb_;	/*!< upper bound on optimization vector */

	Eigen::VectorXd zUpper_; /*!< The upper bound multiplier, used in IPOPT */
	Eigen::VectorXd zLow_; 	/*!< The lower bound multiplier, used in IPOPT */
	Eigen::VectorXd lambda_; /*!< The constraint multiplier, used in IPOPT */

	// Snopt variables
	Eigen::VectorXd xMul_; 		/*!< The optimization variable multiplier, used in SNOPT */
	Eigen::VectorXi xState_;	/*!< The optimization variable state, used in SNOPT */
	Eigen::VectorXd	zMul_;		/*!< The constraint multiplier, used in SNOPT */
	Eigen::VectorXi	zState_;	/*!< The constraint state, used in SNOPT */

	size_t updateCount_; /*!< The number of optimization variable updates */

};

}
}


#endif //CT_OPTCON_NLP_OPTVECTOR_HPP_
