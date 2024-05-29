
#ifndef UTILS_HPP
#define UTILS_HPP

#include "mjpc/tasks/exo/Types.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>
#include <condition_variable>
#include <future>
#include <cmath>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <limits>
#include <mujoco/mujoco.h>
#include "mjpc/utilities.h"
#include "mjpc/tasks/exo/ExoConstants.hpp"


// #include "pinocchio/parsers/urdf.hpp"
// #include "pinocchio/multibody/data.hpp"
// // #include "pinocchio/algorithm/contact-info.hpp"
// #include "pinocchio/spatial/explog.hpp"
// #include "pinocchio/algorithm/kinematics.hpp"
// #include "pinocchio/algorithm/jacobian.hpp"
// #include "pinocchio/algorithm/compute-all-terms.hpp"
// #include "pinocchio/algorithm/joint-configuration.hpp"
// #include "pinocchio/multibody/joint/joints.hpp"
// #include "pinocchio/algorithm/frames.hpp"
// #include "pinocchio/algorithm/center-of-mass.hpp"
// #include "pinocchio/multibody/model.hpp"
// #include "pinocchio/algorithm/aba.hpp"
// #include "pinocchio/algorithm/crba.hpp"
// #include "pinocchio/algorithm/rnea.hpp"
// #include "pinocchio/math/rpy.hpp"


using namespace Exo_t;


namespace taskspace_utils{
	void getY(vector_t& y_act, const mjModel* model, mjData* data,int whichStance);
	void getJacobian(matrix_t& J_y_h, const mjModel* model, mjData* data,int whichStance);
	void getStairY(vector_t& y_act, const mjModel* model, mjData* data,int whichStance);
	void getStairJacobian(matrix_t& J_y_h, const mjModel* model, mjData* data,int whichStance);
}

namespace utils {
	vector_t quatFrom3x3(const Eigen::Matrix3d& m);
	matrix_t R_z(double z);
    vector_t eulRates2omegaXYZ(vector_t eul, vector_t eul_rates);
	vector_t eulRates2omegaZYX(vector_t eul, vector_t eul_rates);
	vector_t omega2eulRates(vector_t eul,vector_t omega);
	vector_t quat2eulXYZ(vector_t quat);
	vector_t quat2eulZYX(vector_t quat);
	quat_t eulerXYZ2quat(vector_t eul);
	quat_t eulerZYX2quat(vector_t eul);

	void eulerZYX(Eigen::Quaterniond &q, Eigen::EulerAnglesZYXd &euler);
	void eulerXYZ(Eigen::Quaterniond &q, Eigen::EulerAnglesXYZd &euler);
	void eulerXYZ(Eigen::Matrix3d &R, Eigen::EulerAnglesXYZd &euler);
	Eigen::Matrix3d skew(Eigen::Vector3d &v);
	void hatmap(const Eigen::Vector3d &w, Eigen::Matrix3d &skew);

	void R_XYZ(vector_t euler_angle, Eigen::Matrix<double,3,3> &R);
	vector_t Rot2eulXYZ(matrix_t R);

	vector_t eulerRatesToAngularVelocity(const Eigen::Vector3d& eulerRates, const Eigen::Vector3d& euler);


	vector_t EulerZYXToQuatWXYZ(vector_t euler_zyx);
	vector_t EulerZYXToQuatXYZW(vector_t euler_zyx);
	vector_t QuatWXYZToEulerZYX(vector_t quat_wxyz);
	vector_t QuatXYZWToEulerZYX(vector_t quat_xyzw);
}


namespace coeffUtils{
	matrix_t reshapeCoeff(int deg, std::vector<scalar_t> coeff_b_vec, std::vector<scalar_t> coeff_jt_vec);
	matrix_t remapSymmetric(matrix_t coeff_mat);
}

namespace matrixUtils{
	matrix_t reshapeMatrix(int numRow, int numCol, std::vector<scalar_t> coeff_jt_vec);
	matrix_t reshapeMatrix(int numRow, int numCol, const double* vec);
	vector_t mapVec2Eigen(std::vector<scalar_t> vec);
	std::vector<scalar_t> flattenMatrix(const matrix_t& mat);
}

namespace jointUtils{
	vector_t remapJoint(vector_t &q);
	vector_t remapActuatedJoint(vector_t &q);
	vector_t convertFrostToPino(vector_t &q_frost);
}





#endif // UTILS_HPP