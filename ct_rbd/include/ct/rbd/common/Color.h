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

#ifndef COLOR_H_
#define COLOR_H_

namespace ct {
namespace common {

class Color
{
public:
	Color() :
		r_(0),
		g_(0),
		b_(0),
		alpha_(1)
	{}

	Color(double r, double g, double b, double alpha = 1) :
		r_(r),
		g_(g),
		b_(b),
		alpha_(alpha)
	{}

	Color(double grayBrightness, double alpha = 1) :
		r_(grayBrightness),
		g_(grayBrightness),
		b_(grayBrightness),
		alpha_(alpha)
	{}

	// predefined colors
	static const Color RED;
	static const Color BLUE;
	static const Color GREEN;
	static const Color YELLOW;
	static const Color ORANGE;
	static const Color MAGENTA;
	static const Color PINK;
	static const Color CYAN;
	static const Color BROWN;
	static const Color WHITE;
	static const Color BLACK;
	static const Color GREY;


	double& r() { return r_; }
	double& g() { return g_; }
	double& b() { return b_; }

	double& alpha() { return alpha_; }


protected:
	double r_;
	double g_;
	double b_;
	double alpha_;
};




} // namespace common
} // namespace ct


#endif /* COLOR_H_ */
