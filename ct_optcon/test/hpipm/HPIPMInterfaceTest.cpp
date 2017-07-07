/*
 * HPIPMInterfaceTest.cpp
 *
 *  Created on: Jul 7, 2017
 *      Author: neunertm
 */

#include <ct/optcon/optcon.h>

using namespace ct::core;

static const int state_dim = 8;
static const int control_dim = 3;

void dmcopy(int row, int col, double *A, int lda, double *B, int ldb)
	{
	int i, j;
	for(j=0; j<col; j++)
		{
		for(i=0; i<row; i++)
			{
			B[i+j*ldb] = A[i+j*lda];
			}
		}
	}

class LinkedMasses : public LinearSystem<state_dim, control_dim>
{
public:
	state_matrix_t A_;
	state_control_matrix_t B_;

	LinkedMasses()
	{
		A_.setZero();
		B_.setZero();

		static const int pp = state_dim/2; // number of masses

		Eigen::Matrix<double, pp, pp> TEigen; TEigen.setZero();

		double *T = TEigen.data();
		int ii;
		for(ii=0; ii<pp; ii++) T[ii*(pp+1)] = -2;
		for(ii=0; ii<pp-1; ii++) T[ii*(pp+1)+1] = 1;
		for(ii=1; ii<pp; ii++) T[ii*(pp+1)-1] = 1;

		Eigen::Matrix<double, pp, pp> ZEigen; ZEigen.setZero();
		double *Z = ZEigen.data();

		Eigen::Matrix<double, pp, pp> IEigen; IEigen.setIdentity();
		double *I = IEigen.data();

		double *Ac = A_.data();
		dmcopy(pp, pp, Z, pp, Ac, state_dim);
		dmcopy(pp, pp, T, pp, Ac+pp, state_dim);
		dmcopy(pp, pp, I, pp, Ac+pp*state_dim, state_dim);
		dmcopy(pp, pp, Z, pp, Ac+pp*(state_dim+1), state_dim);

		Eigen::Matrix<double, control_dim, control_dim> InuEigen; InuEigen.setIdentity();
		double *Inu = InuEigen.data();

		double *Bc = B_.data();
		dmcopy(control_dim, control_dim, Inu, control_dim, Bc+pp, state_dim);

		std::cout << "Ac "<< std::endl<<A_<<std::endl <<std::endl;
		std::cout << "Bc "<< std::endl<<B_<<std::endl <<std::endl;
	}


	const state_matrix_t& getDerivativeState(const StateVector<state_dim>& x, const ControlVector<control_dim>& u, const double t = 0.0) override
	{
		return A_;
	}

	const state_control_matrix_t& getDerivativeControl(const StateVector<state_dim>& x, const ControlVector<control_dim>& u, const double t = 0.0) override
	{
		return B_;
	}

	LinkedMasses* clone() const override
	{
		return new LinkedMasses();
	};
};

int main(int argc, char* argv[])
{
	LinkedMasses system;

	int N = 5;

	double dt = 0.5;

	ct::optcon::HPIPMInterface<state_dim, control_dim> interface(N);
	StateVector<state_dim> x0;

	StateMatrix<state_dim> Q;
	Q.setIdentity();
	Q *= 2.0;
	ControlMatrix<control_dim> R;
	R.setIdentity();
	R *= 2*2.0;

	StateVector<state_dim> xNom;
	xNom.setConstant(-0.1);

	ControlVector<control_dim> uNom;
	uNom.setConstant(-0.1);

	ct::optcon::CostFunctionQuadraticSimple<state_dim, control_dim> costFunction(Q, R,
			xNom, uNom,
			xNom, Q);

	interface.setLinearProblem(x0, system, costFunction, dt);

	interface.solve();

	return 1;
}
