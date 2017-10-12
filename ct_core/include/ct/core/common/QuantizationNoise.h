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

#pragma once

namespace ct {
namespace core {

//! Quantization of data
/*!
 * This class provides basic functionality to simulate quantization of (sensor) data.
 * Data input can be either rounded down or rounded to a given quantization interval.
 * It tracks the rest of the quantization for the subsequent call
 */
class QuantizationNoise
{
public:
	//! The quanitization method
	enum QuantizationMethod {
		FLOOR, /*!< Enum to round down  */
		ROUND /*!< Enum to round */
	};

	//! Default constructor
	/*!
	 * @param bias A bias to apply after the quantization (will not be quantized!)
	 * @param quantizationInterval The quantization interval to quantize to
	 * @param method The quantization method
	 */
	QuantizationNoise(double bias = 0.0, double quantizationInterval = 1.0, QuantizationMethod method = FLOOR) :
		quantizationInterval_(quantizationInterval),
		quantizationRest_(1, 0.0),
		bias_(bias),
		quantizationMethod_(method)
	{
	}

	//! Applies quantization to a value
	/*!
	 * This applies quantization to a value and stores the rest. The given index
	 * serves to track the rest of the quantization for the next call. The index
	 * is not a time index but simply an 'ID'.
	 * @param value The value to quantize
	 * @param index The identifier
	 */
	void noisify(double& value, size_t index = 0) {

		if (index >= quantizationRest_.size())
			quantizationRest_.resize(index, 0);

		double currentValue = value + quantizationRest_[index];
		double nQuantsDouble = currentValue/quantizationInterval_;
		int nQuants;

		switch(quantizationMethod_)
		{
			case FLOOR:
			{
				nQuants = std::floor(nQuantsDouble);
				break;
			}
			case ROUND:
			{
				nQuants = std::round(nQuantsDouble);
				break;
			}
			default:
			{
				throw std::runtime_error("Unknown quantization method chosen");
			}
		}

		double outputValue = nQuants * quantizationInterval_;

		quantizationRest_[index] = currentValue - outputValue;

		value = outputValue +  bias_;
	}

	//! Applies quantization to a vector
	/*!
	 * This applies quantization to a vector and stores the rest. It uses
	 * the entry in the vector as an identifier to store the rest of the value
	 * @param value The value to quantize
	 */
	template <size_t size>
	void noisify(Eigen::Matrix<double, size, 1>& value) {
		for (size_t i=0; i<size; i++)
		{
			noisify(value(i), i);
		}
	}

	//! Resets the quantization
	/*!
	 * Deletes the quantization rest to reset the quantization
	 */
	void reset()
	{
		quantizationRest_.clear();
	}


private:
	double quantizationInterval_; /*!< interval for quantization */
	std::vector<double> quantizationRest_; /*!< tracks the rest of the quantization */
	double bias_; /*!< a bias to apply */
	QuantizationMethod quantizationMethod_; /*!< the quantization method */

};


} // namespace ct
} // namespace core



