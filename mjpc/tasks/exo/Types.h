#ifndef TYPES_H
#define TYPES_H

// #pragma once
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <ostream>
#include <vector>

namespace Exo_t {

/** size_t trajectory type. */
using size_array_t = std::vector<size_t>;
/** Array of size_t trajectory type. */
using size_array2_t = std::vector<size_array_t>;

/** Scalar type. */
using scalar_t = double;
/** Scalar trajectory type. */
using scalar_array_t = std::vector<scalar_t>;
/** Array of scalar trajectory type. */
using scalar_array2_t = std::vector<scalar_array_t>;
/** Array of arrays of scalar trajectory type. */
using scalar_array3_t = std::vector<scalar_array2_t>;

/** Dynamic-size vector type. */
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
/** Static-size vector type. */
using vector_2t = Eigen::Matrix<scalar_t, 2, 1>;
using vector_3t = Eigen::Matrix<scalar_t, 3, 1>;
using vector_4t = Eigen::Matrix<scalar_t, 4, 1>;
using vector_8t = Eigen::Matrix<scalar_t, 8, 1>;
using vector_12t = Eigen::Matrix<scalar_t,12, 1>;
/** Dynamic vector's trajectory type. */
using vector_array_t = std::vector<vector_t>;
/** Array of dynamic vector's trajectory type. */
using vector_array2_t = std::vector<vector_array_t>;
/** Array of arrays of dynamic vector trajectory type. */
using vector_array3_t = std::vector<vector_array2_t>;

/** Dynamic-size row vector type. */
using row_vector_t = Eigen::Matrix<scalar_t, 1, Eigen::Dynamic>;

/** Dynamic-size matrix type. */
using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
/** Static-size Matrix type. */
using matrix_3t = Eigen::Matrix<scalar_t, 3, 3>;
/** Dynamic matrix's trajectory type. */
using matrix_array_t = std::vector<matrix_t>;
/** Array of dynamic matrix's trajectory type. */
using matrix_array2_t = std::vector<matrix_array_t>;
/** Array of arrays of dynamic matrix trajectory type. */
using matrix_array3_t = std::vector<matrix_array2_t>;

using quat_t = Eigen::Quaternion<scalar_t>;

/** Eigen scalar type. */
using eigen_scalar_t = Eigen::Matrix<scalar_t, 1, 1>;
/** Eigen scalar trajectory type. */
using eigen_scalar_array_t = std::vector<eigen_scalar_t>;
/** Array of eigen scalar trajectory type. */
using eigen_scalar_array2_t = std::vector<eigen_scalar_array_t>;
/** Array of arrays of eigen scalar trajectory type. */
using eigen_scalar_array3_t = std::vector<eigen_scalar_array2_t>;

} 

#endif // TYPES_H