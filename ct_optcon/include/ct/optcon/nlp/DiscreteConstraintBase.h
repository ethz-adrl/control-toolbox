/***********************************************************************************
Copyright (c) 2016, Agile & Dexterous Robotics Lab, ETH ZURICH. All rights reserved.

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

#ifndef CT_OPTCON_NLP_CONSTRAINT_BASE_H_
#define CT_OPTCON_NLP_CONSTRAINT_BASE_H_

namespace ct {
namespace optcon {

/**
 * @ingroup    NLP
 *
 * @brief      Implements an abstract base class from which all the discrete
 *             custom NLP constraints should derive
 */
class DiscreteConstraintBase{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 * @brief      Default constructor
	 */
	DiscreteConstraintBase(){}

	/**
	 * @brief      Destructor
	 */
	virtual ~DiscreteConstraintBase(){}

	/**
	 * @brief          Evaluates to constraint
	 *
	 * @param[in, out] val       The large optimiz
	 * @param[in]      startInd  The start ind
	 *
	 * @return         The evaluation.
	 */

	virtual Eigen::VectorXd eval() = 0;
	// virtual size_t getEvaluation(Eigen::Map<Eigen::VectorXd>& val, size_t startInd) = 0;

	virtual Eigen::VectorXd evalJacobian() = 0;
	// virtual size_t evalConstraintJacobian(Eigen::Map<Eigen::VectorXd>& val, size_t startInd) = 0;

	virtual size_t getNumNonZerosJacobian() = 0;

	virtual size_t getConstraintSize() = 0;

	virtual void genSparsityPattern(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) = 0;

	virtual Eigen::VectorXd getLowerBound() = 0;
	virtual Eigen::VectorXd getUpperBound() = 0;

	// virtual void getLowerBound(Eigen::VectorXd& c_lb) = 0;

	// virtual void getUpperBound(Eigen::VectorXd& c_ub) = 0;


	// const size_t c_index() const {return indexTotal_;}

	// virtual void initialize(size_t c_index)
	// {
	// 	indexTotal_ = c_index;
	// }

protected:

	size_t genDiagonalIndices(
			const size_t col_start,
			const size_t num_elements,
			Eigen::VectorXi& iRow_vec,
			Eigen::VectorXi& jCol_vec,
			const size_t indexNumber);

	size_t genBlockIndices(
			const size_t col_start,
			const size_t num_rows,
			const size_t num_cols,
			Eigen::VectorXi& iRow_vec,
			Eigen::VectorXi& jCol_vec,
			const size_t indexNumber);

};


/* helper function for indexing the non-zero elements in the constraint jacobian.
 * Resizes the Eigenvectors to the number of elements in the diagonal defined by the starting
 * values and the number of elements. This generates a diagonal which goes from the top left corner
 * to the bottom right corner */
size_t DiscreteConstraintBase::genDiagonalIndices(
		const size_t col_start,							// matrix col index where we start inserting
		const size_t num_elements, 						// number of diagonal elements the diag matrix shall have
		Eigen::VectorXi& iRow_vec,			// big ipopt row index vector
		Eigen::VectorXi& jCol_vec,			// big ipopt col index vector
		const size_t indexNumber						// where we insert the generated indices into the big ipopt index vectors
)
{
	Eigen::VectorXi new_row_indices;
	Eigen::VectorXi new_col_indices;
	new_row_indices.resize(num_elements);
	new_col_indices.resize(num_elements);

	size_t count = 0;

	for(size_t i = 0; i < num_elements; ++i){
		new_row_indices(count) = i;
		new_col_indices(count) = col_start+i;
		count++;
	}

	assert(count == num_elements);

	iRow_vec.segment(indexNumber, num_elements) = new_row_indices;
	jCol_vec.segment(indexNumber, num_elements) = new_col_indices;

	return count;
}


/* helper function for indexing the non-zero elements in the constraint jacobian.
 * Resizes the Eigenvectors to the number of elements in the block defined by the starting
 * values and the widths and enters the row- and column indices into the vectors. */
size_t DiscreteConstraintBase::genBlockIndices(
		const size_t col_start,                   	// matrix col index where we start inserting
		const size_t num_rows, 						// number of rows the block matrix shall have
		const size_t num_cols,						// number of cols the block matrix shall have
		Eigen::VectorXi& iRow_vec,		// big ipopt row index vector
		Eigen::VectorXi& jCol_vec,	    // big ipopt col index vector
		const size_t indexNumber)					// where we insert the generated indices into the big ipopt index vectors
{
	size_t num_gen_indices = num_rows*num_cols;

	Eigen::VectorXi new_row_indices;
	Eigen::VectorXi new_col_indices;
	new_row_indices.resize(num_gen_indices);
	new_col_indices.resize(num_gen_indices);

	size_t count = 0;

	for(size_t row = 0; row <num_rows; ++row){
		for(size_t col = col_start; col <col_start+num_cols; ++col){
			new_row_indices(count) = row;
			new_col_indices(count) = col;
			count++;
		}
	}

	assert(count == num_gen_indices);

	iRow_vec.segment(indexNumber, num_gen_indices) = new_row_indices;
	jCol_vec.segment(indexNumber, num_gen_indices) = new_col_indices;

	return num_gen_indices;
}


} // namespace optcon
} // namespace ct

#endif //CT_OPTCON_NLP_CONSTRAINT_BASE_H_
