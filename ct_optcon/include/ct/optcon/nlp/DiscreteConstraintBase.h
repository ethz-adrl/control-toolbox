#ifndef DMS_CONSTRAINT_BASE_HPP
#define DMS_CONSTRAINT_BASE_HPP

namespace ct {
namespace optcon {

class DiscreteConstraintBase{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	DiscreteConstraintBase(){}

	~DiscreteConstraintBase(){}

	//Writes constraint evaluation inside Optimization Vector
	virtual size_t getEvaluation(Eigen::Map<Eigen::VectorXd>& val, size_t count) = 0;

	//Writes constraint evaluation inside Optimization Vector
	virtual size_t evalConstraintJacobian(Eigen::Map<Eigen::VectorXd>& val, size_t count) = 0;

	virtual size_t getNumNonZerosJacobian() = 0;

	virtual size_t genSparsityPattern(
		Eigen::Map<Eigen::VectorXi>& iRow_vec,
		Eigen::Map<Eigen::VectorXi>& jCol_vec,
		size_t indexNumber) = 0;

	virtual void getLowerBound(Eigen::VectorXd& c_lb) = 0;

	virtual void getUpperBound(Eigen::VectorXd& c_ub) = 0;

	virtual size_t getConstraintSize() = 0;

	const size_t c_index() const {return indexTotal_;}

	virtual void initialize(size_t c_index)
	{
		indexTotal_ = c_index;
	}

protected:

	size_t genDiagonalIndices(
			const size_t row_start,
			const size_t col_start,
			const size_t num_elements,
			Eigen::Map<Eigen::VectorXi>& iRow_vec,
			Eigen::Map<Eigen::VectorXi>& jCol_vec,
			const size_t indexNumber);

	size_t genBlockIndices(
			const size_t row_start,
			const size_t col_start,
			const size_t num_rows,
			const size_t num_cols,
			Eigen::Map<Eigen::VectorXi>& iRow_vec,
			Eigen::Map<Eigen::VectorXi>& jCol_vec,
			const size_t indexNumber);

	size_t evalIblock(
			Eigen::Map<Eigen::VectorXd>& val,
			size_t indexNumber,
			size_t block_length);

	size_t indexTotal_; 	// absolute starting index in the large constraint vector
};


/* helper function for indexing the non-zero elements in the constraint jacobian.
 * Resizes the Eigenvectors to the number of elements in the diagonal defined by the starting
 * values and the number of elements. This generates a diagonal which goes from the top left corner
 * to the bottom right corner */
size_t DiscreteConstraintBase::genDiagonalIndices(
		const size_t row_start,							// matrix row index where we start inserting
		const size_t col_start,							// matrix col index where we start inserting
		const size_t num_elements, 						// number of diagonal elements the diag matrix shall have
		Eigen::Map<Eigen::VectorXi>& iRow_vec,			// big ipopt row index vector
		Eigen::Map<Eigen::VectorXi>& jCol_vec,			// big ipopt col index vector
		const size_t indexNumber						// where we insert the generated indices into the big ipopt index vectors
)
{
	Eigen::VectorXi new_row_indices;
	Eigen::VectorXi new_col_indices;
	new_row_indices.resize(num_elements);
	new_col_indices.resize(num_elements);

	size_t count = 0;

	for(size_t i = 0; i < num_elements; ++i){
		new_row_indices(count) = row_start+i;
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
		const size_t row_start,                   	// matrix row index where we start inserting
		const size_t col_start,                   	// matrix col index where we start inserting
		const size_t num_rows, 						// number of rows the block matrix shall have
		const size_t num_cols,						// number of cols the block matrix shall have
		Eigen::Map<Eigen::VectorXi>& iRow_vec,		// big ipopt row index vector
		Eigen::Map<Eigen::VectorXi>& jCol_vec,	    // big ipopt col index vector
		const size_t indexNumber)					// where we insert the generated indices into the big ipopt index vectors
{
	size_t num_gen_indices = num_rows*num_cols;

	Eigen::VectorXi new_row_indices;
	Eigen::VectorXi new_col_indices;
	new_row_indices.resize(num_gen_indices);
	new_col_indices.resize(num_gen_indices);

	size_t count = 0;

	for(size_t row = row_start; row <row_start+num_rows; ++row){
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


// blockSize denotes the number of elements on the diagonal
size_t DiscreteConstraintBase::evalIblock(Eigen::Map<Eigen::VectorXd>& val, size_t indexNumber, size_t block_length)
{
	// call method to get derivative
	Eigen::MatrixXd mat = Eigen::MatrixXd::Identity(block_length, block_length);

	// fill into value vector with correct indexing
	size_t count = indexNumber;

	for(size_t element = 0; element < block_length; ++element){
		val(count) = mat(element, element);
		count++;
	}

	return count;
}

} // namespace optcon
} // namespace ct

#endif
