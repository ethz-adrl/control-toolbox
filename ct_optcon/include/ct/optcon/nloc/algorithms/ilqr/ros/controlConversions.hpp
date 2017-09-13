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

#ifndef CONTROLCONVERSIONS_HPP_
#define CONTROLCONVERSIONS_HPP_

#include <ilqg/Dimensions.hpp>

#include <ilqg/iLQG.h>

template <class Derived>
void matrixEigenToMsg(const Eigen::MatrixBase<Derived> &e, std_msgs::Float64MultiArray &m)
{
  if (m.layout.dim.size() != 2)
    m.layout.dim.resize(2);
  m.layout.dim[0].stride = e.rows() * e.cols();
  m.layout.dim[0].size = e.rows();
  m.layout.dim[1].stride = e.cols();
  m.layout.dim[1].size = e.cols();
  if ((int)m.data.size() != e.size())
    m.data.resize(e.size());
  int ii = 0;
  for (int i = 0; i < e.rows(); ++i)
    for (int j = 0; j < e.cols(); ++j)
      m.data[ii++] = e.coeff(i, j);
}

template <class Derived>
void msgToMatrixEigen(const std_msgs::Float64MultiArray &m, Eigen::MatrixBase<Derived> &e)
{
	assert(m.layout.dim[0].size == e.rows());
	assert(m.layout.dim[1].size == e.cols());

	int ii = 0;
	for (int i = 0; i < e.rows(); ++i)
	  for (int j = 0; j < e.cols(); ++j)
		e(i, j) = m.data[j + i*e.cols()];
}

template <typename DIM>
void controlToROSMsg(const typename DIM::control_vector_array_t& u_ff, const typename DIM::control_feedback_array_t& u_fb, size_t K, double dt, ilqg::iLQG& msg)
{
	msg.dt = dt;
	msg.t_end = K * dt;
	msg.time_steps = K;

	assert(u_ff.size() >= K);
	assert(u_fb.size() >= K);

	msg.control.resize(K);

	for (size_t i=0; i<K; i++)
	{
		msg.control[i].u_ff.resize(DIM::CONTROL_SIZE);
		for (size_t j=0; j<DIM::CONTROL_SIZE; j++)
		{
			msg.control[i].u_ff[j].data = u_ff[i](j);
		}

		matrixEigenToMsg(u_fb[i], msg.control[i].u_fb);
	}
}



template <typename DIM>
void ROSMsgtoControl(const ilqg::iLQG& msg, typename DIM::control_vector_array_t& u_ff, typename DIM::control_feedback_array_t& u_fb, size_t& K, double& dt)
{
	dt = msg.dt;
	K = msg.time_steps;

	assert(std::abs(K * dt - msg.t_end) < 1e-6);

	assert(msg.control.size() == K);

	// resize and fill feed forward
	u_ff.resize(K);
	u_fb.resize(K);

	for (size_t i=0; i<K; i++)
	{
		for (size_t j=0; j<DIM::CONTROL_SIZE; j++)
		{
			u_ff[i](j) = msg.control[i].u_ff[j].data;
		}
		msgToMatrixEigen(msg.control[i].u_fb, u_fb[i]);
	}
}



#endif /* CONTROLCONVERSIONS_HPP_ */
