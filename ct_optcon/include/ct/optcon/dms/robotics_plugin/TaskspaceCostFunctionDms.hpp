#ifndef DMS_TASKSPACECOSTFUNCTION_HPP_
#define DMS_TASKSPACECOSTFUNCTION_HPP_


#include <ct/optcon/dms/dms_core/DmsDimensions.hpp>

#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM>
class TaskspaceCostFunctionDms : public CostFunctionQuadratic< STATE_DIM, CONTROL_DIM >
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DmsDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;
	typedef typename DIMENSIONS::control_matrix_t control_matrix_t;

	TaskspaceCostFunctionDms(const state_matrix_t& Q, const control_matrix_t& R,
		const control_vector_t& u_nominal,
		const state_vector_t& x_final, const state_matrix_t& Q_final,
		const double T_penalty,
		std::function<void (const state_vector_t&, Eigen::Vector3d&)> stateToPosition,
		std::function<void (const state_vector_t&, Eigen::Matrix<double, 3, STATE_DIM>&)> stateToPositionJacobi,
		const dms_settings settings
		) :
			Q_(Q),
			R_(R),
			u_nominal_(u_nominal),
			x_final_(x_final),
			x_nominal_(x_final),
			Q_final_(Q_final),
			T_penalty_(T_penalty),
			stateToPosition_(stateToPosition),
			stateToPositionJacobi_(stateToPositionJacobi),
			x_deviation_(state_vector_t::Zero()),
			u_deviation_(control_vector_t::Zero())
	{
		pos_.setZero();
		pos_nominal_.setZero();
		pos_final_.setZero();

		stateToPosition(x_nominal_, pos_nominal_);
		stateToPosition(x_final, pos_final_);

		Q_block_ = Q_.block(0,0,3,3);
		Q_final_block_ = Q_final_.block(0,0,3,3);
	}


	TaskspaceCostFunctionDms(const TaskspaceCostFunctionDms& arg):
		ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>(arg),
		Q_(arg.Q_),
		R_(arg.R_),
		u_nominal_(arg.u_nominal_),
		x_final_(arg.x_final_),
		x_nominal_(arg.x_nominal_),
		Q_final_(arg.Q_final_),
		T_penalty_(arg.T_penalty_),
		stateToPosition_(arg.stateToPosition_),
		stateToPositionJacobi_(arg.stateToPositionJacobi_),
		x_deviation_(arg.x_deviation_),
		u_deviation_(arg.u_deviation_),
		pos_nominal_(arg.pos_nominal_),
		pos_final_(arg.pos_final_),
		Q_block_(arg.Q_block_),
		Q_final_block_(arg.Q_final_block_)
		{}

	std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>> clone() const override
	{
		return std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>> (new TaskspaceCostFunctionDms(*this));
	}


	virtual ~TaskspaceCostFunctionDms() {}

	virtual void updatexFinal(const state_vector_t& x_final) { x_final_ = x_final; }

	virtual void setCurrentStateAndControl(const state_vector_t& x, const control_vector_t& u)
	{
		this->x_ = x;
		this->u_ = u;

		pos_.setZero();
		jacobi_.setZero();
		stateToPosition_(x, pos_);
		stateToPositionJacobi_(x, jacobi_);

		this->u_deviation_ = u - u_nominal_;
	}

	//Only considers topleft corner of Q matrix. This corner represents [x,y,z] position costs when using this costfunction
	virtual scalar_t evaluateIntermediate()
	{
		scalar_t costQ = 0.5 * (pos_ - pos_nominal_).transpose() * Q_block_ * (pos_ - pos_nominal_);

		scalar_t costR = 0.5 * u_deviation_.transpose() * R_ * u_deviation_;
		scalar_t costT = T_penalty_;

		assert(costQ == costQ);
		assert(costR == costR);
		assert(costT == costT);

		return costQ + costR + costT;
	}


	virtual state_vector_t stateDerivativeIntermediate()
	{
		state_vector_t stateDerivative; stateDerivative.setZero();
		stateDerivative = jacobi_.transpose() * Q_block_ * (pos_ - pos_nominal_);

		// std::cout << "Jacobi: " << jacobi_ << std::endl;

		assert(stateDerivative == stateDerivative);
		return  stateDerivative;
	}

	virtual control_vector_t controlDerivativeIntermediate()
	{
		control_vector_t controlDerivative = R_ * u_deviation_;
		assert(controlDerivative == controlDerivative);
		return controlDerivative;
	}


	virtual scalar_t evaluateTerminal()
	{
		scalar_t costTer = 0.5 * (pos_ - pos_final_).transpose() * Q_final_block_ * (pos_ - pos_final_);

		assert(costTer == costTer);

		return costTer;
	}

	virtual state_vector_t stateDerivativeTerminal()
	{
		state_vector_t terminalDerivative; terminalDerivative.setZero();
		terminalDerivative = jacobi_.transpose() * Q_final_block_ * (pos_ - pos_final_);

		assert(terminalDerivative==terminalDerivative);

		return terminalDerivative;
	}

protected:

	state_matrix_t Q_;
	control_matrix_t R_;
	control_vector_t u_nominal_;
	state_vector_t x_final_;
	state_vector_t x_nominal_;
	state_matrix_t Q_final_;

	scalar_t T_penalty_;
	std::function<void (const state_vector_t&, Eigen::Vector3d&)> stateToPosition_;
	std::function<void (const state_vector_t&, Eigen::Matrix<double, 3, STATE_DIM>&)> stateToPositionJacobi_;

	state_vector_t x_deviation_;
	control_vector_t u_deviation_;

	Eigen::Vector3d pos_;
	Eigen::Vector3d pos_nominal_;
	Eigen::Vector3d pos_final_;
	Eigen::Matrix<double, 3, STATE_DIM> jacobi_;

	Eigen::Matrix3d Q_block_;
	Eigen::Matrix3d Q_final_block_;

};

} // namespace optcon
} // namespace ct

#endif
