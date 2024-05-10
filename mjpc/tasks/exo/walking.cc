#include "mjpc/tasks/exo/walking.h"
#include "mjpc/tasks/exo/Types.h"
#include "mjpc/tasks/exo/utils.hpp"
#include <string>

#include <mujoco/mujoco.h>
#include "mjpc/utilities.h"
#include <absl/random/random.h>

#include "yaml-cpp/yaml.h"
#include <algorithm>
#include "mjpc/tasks/exo/ExoConstants.hpp"

namespace mjpc::exo {

using namespace Exo_t;

void walking::GetNominalAction(const mjModel* model, double* action, mjData* kin_data,double time) const {
                              // std::cout << "Getting default Walkinging pos" << std::endl;


     //instead of use the qpos from state, use the qpos from evaluate jt bezier
    vector_t q_init,dq_init;
    std::tie(q_init,dq_init) = walking::evalJtBezier(time-initial_t0,coeff,coeff_remap,whichStance, walkStepDur);

    double scale[12] = {0.1};
    bool jt_space = false;
    if(jt_space){
        for (int i = 0; i < 12; i++) {
            action[i] = scale[i] *action[i] + q_init[i+7];
        }

        for(int i=0; i<12;i++){
            action[i+12] = scale[i] *action[i+12] + dq_init[i+6];
        }
    }
    else{
        std::cout << "IK mode" << std::endl;

        matrix_t J_y = matrix_t::Zero(12,18);
        matrix_t J_h = matrix_t::Zero(6,18);


        Eigen::VectorXd yDesFull = Eigen::VectorXd::Zero(18);

        //loop through the first 12 element
        vector_t yd,dyd = vector_t::Zero(12);
        std::tie(yd,dyd) = evalTaskBezier(time-initial_t0,coeff_task,coeff_task_remap,whichStance, walkStepDur);

        // yd[2] = yd[2] - 0.005;
        yDesFull.segment(0,12) = yd;
        dyd.segment(0,12) = dyd;
    
        for(int i=0; i<12;i++){
        
            yDesFull[i] = yd[i] + action[i];
            dyd[i] = dyd[i] + action[i+12];
        }

        Eigen::VectorXd err;
        bool success = false;
        int maxIKIterations = 3;
        double dampingFactor = 5e-3;
    
    

        // //update data using state, qpos, qvel
        for (int i = 0; i < model->nq; i++) {
            kin_data->qpos[i] = q_init[i];
        }
        // set qvel, qacc to be zero
        for (int i = 0; i < model->nv; i++) {
            kin_data->qvel[i] = 0;
            kin_data->qacc[i] = 0;
        }

    
        vector_t yCur = vector_t::Zero(18);
        matrix_t J = matrix_t::Zero(18,18);
        for (int i = 0; i < maxIKIterations; ++i) {
            //print qpos
            
            mj_kinematics(model, kin_data);
            mj_comPos(model, kin_data);


            taskspace_utils::getY(yCur,model,kin_data, whichStance);

            err = yDesFull - yCur;
            if (err.cwiseAbs().maxCoeff() < 1e-3) {
                success = true;
                std::cout << "IK converged in " << i << " iterations" << std::endl;
                break;
            }

            taskspace_utils::getJacobian(J,model,kin_data, whichStance);
            Eigen::MatrixXd JJt = J * J.transpose() + Eigen::MatrixXd::Identity(J.rows(), J.rows()) * dampingFactor;
            Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse(); // Compute the pseudo-inverse using COD
            Eigen::VectorXd deltaQ = J_pinv * err; // Directly use the pseudo-inverse to compute deltaQ
    
            

            for (int j = 0; j < model->nv; j++) {
                kin_data->qvel[j] = deltaQ[j];
            }

            mj_integratePos(model, kin_data->qpos, kin_data->qvel, 1);
        }
  
        // std::cout << "IK success: " << success << std::endl;
        // std::cout << "IK error: " << err.norm() << std::endl;
    // if(!success){
    //     std::cout << "IK failed" << std::endl;
    //     // std::terminate();
    // }

        matrix_t J_output = J.block(0,0,12,18);
        vector_t dq_ik = J_output.transpose()*(J_output * J_output.transpose()).inverse() * dyd;
        // overide action with current qpos
        for (int i = 0; i < 12; i++) {
        action[i] = kin_data->qpos[i+7];
        }

        for(int i=0; i<12;i++){
        action[i+12] = dq_ik[i+6];
        }
    }

}


std::string walking::XmlPath() const {
  return GetModelPath("exo/plan_task.xml");
}


std::string walking::Name() const { return "Exo Walking"; }

void walking::ResidualFn::loadGoalJtPosition(){
  std::string FilePath = GetModelPath("exo/walkConfig.yaml");
	YAML::Node config = YAML::LoadFile(FilePath);

  
	// Read coefficients from yaml
	std::vector<scalar_t> coeff_jt_vec = config["coeff_jt"].as<std::vector<scalar_t>>();
	std::vector<scalar_t> coeff_b_vec = config["coeff_b"].as<std::vector<scalar_t>>();
  std::vector<scalar_t> coeff_task_vec = config["coeff_output"].as<std::vector<scalar_t>>();


	walkStepDur = config["step_dur"].as<scalar_t>();
	
	// Get matrix of coefficients and remapped coefficients
	int deg = 7;
	coeff = coeffUtils::reshapeCoeff(deg, coeff_b_vec, coeff_jt_vec);
	coeff_remap = coeffUtils::remapSymmetric(coeff);

    // coeff_task = matrixUtils::reshapeMatrix(12,deg+1,coeff_task_vec);
    // coeff_task_remap = coeffUtils::remapSymmetric(coeff_task);
   
  coeff_task = matrixUtils::reshapeMatrix(12,deg+1,coeff_task_vec);
  coeff_task_remap = coeff_task;
  vector_t output_sign = vector_t::Ones(12);
	output_sign << 1,-1,1,-1,1,-1,1,-1,1,-1,1,-1;
	for (int j=0;j<12;j++){ 
	 	coeff_task_remap.block(j,0,1,8) = output_sign[j] * coeff_task_remap.block(j,0,1,8);
	}
}

void walking::loadGoalJtPosition(){
    std::string FilePath = GetModelPath("exo/walkConfig.yaml");
	YAML::Node config = YAML::LoadFile(FilePath);

	// Read coefficients from yaml
	std::vector<scalar_t> coeff_jt_vec = config["coeff_jt"].as<std::vector<scalar_t>>();
	std::vector<scalar_t> coeff_b_vec = config["coeff_b"].as<std::vector<scalar_t>>();
    std::vector<scalar_t> coeff_task_vec = config["coeff_output"].as<std::vector<scalar_t>>();
  
  
	walkStepDur = config["step_dur"].as<scalar_t>();
	step_dur = walkStepDur;
    whichStance = Right;
	// Get matrix of coefficients and remapped coefficients
	int deg = 7;
	coeff = coeffUtils::reshapeCoeff(deg, coeff_b_vec, coeff_jt_vec);
	coeff_remap = coeffUtils::remapSymmetric(coeff);

    coeff_task = matrixUtils::reshapeMatrix(12,deg+1,coeff_task_vec);
    coeff_task_remap = coeff_task;
    vector_t output_sign = vector_t::Ones(12);
        output_sign << 1,-1,1,-1,1,-1,1,-1,1,-1,1,-1;
        for (int j=0;j<12;j++){ 
            coeff_task_remap.block(j,0,1,8) = output_sign[j] * coeff_task_remap.block(j,0,1,8);
        }

}



std::tuple<vector_t,vector_t> walking::evalJtBezier(double time,matrix_t coeff_, matrix_t coeff_remap_,int whichStance, scalar_t walkStepDur_) {

  // Calculate the phase variable within the current cycle
  scalar_t phaseVar = (time) / walkStepDur_;

	matrix_t coeff_cur = matrix_t::Zero(18,8);
	switch(whichStance){
		case Left:{//left stance; frost assume right stance so need to use the remap coefficient
			coeff_cur  = coeff_remap_; 
            // std::cout  << "left stance" << std::endl;
			break;}
		case Right:{
			coeff_cur = coeff_;
            // std::cout  << "right stance" << std::endl;
			break;}
	}
	vector_t jt_config = vector_t::Zero(18);
	bezier_tools::bezier(coeff_cur,phaseVar,jt_config);
	vector_t djt_config = vector_t::Zero(18);
	bezier_tools::dbezier(coeff_cur,phaseVar,djt_config); 

  vector_t q_des = vector_t::Zero(19);
  vector_t dq_des = vector_t::Zero(18);
	// Convert position to have quaternion representation for Free Flyer
	quat_t quat = utils::eulerXYZ2quat(jt_config.segment(3,3));
  vector_t quat_coeff = quat.coeffs();//eigen wxyz but return coeff in xyzw
	q_des << jt_config.segment(0,3),quat_coeff[3],quat_coeff[0],quat_coeff[1],quat_coeff[2],jt_config.segment(6,12);

	// Populate Desired Velocities
	vector_t dq_des_f = djt_config * (1/walkStepDur_);
	vector_t omega = utils::eulRates2omegaXYZ(jt_config.segment(3,3),dq_des_f.segment(3,3));
	dq_des << dq_des_f.segment(0,3),omega,dq_des_f.segment(6,12);

  return std::tie(q_des,dq_des);
}

std::tuple<vector_t,vector_t> walking::evalTaskBezier(double time,matrix_t coeff_, matrix_t coeff_remap_,int whichStance, scalar_t walkStepDur_){
  // Get the phase variable

    // Calculate the number of complete cycles
//   int numCycle = static_cast<int>(time / walkStepDur_);

  // Calculate the phase variable within the current cycle
  scalar_t phaseVar = (time) / walkStepDur_;


	matrix_t coeff_cur = matrix_t::Zero(12,8);
	switch(whichStance){
		case Left:{//left stance; frost assume right stance so need to use the remap coefficient
			coeff_cur  = coeff_remap_; 
            // std::cout  << "left stance" << std::endl;
			break;}
		case Right:{
			coeff_cur = coeff_;
            // std::cout << "right stance" << std::endl;
			break;}
	}
    // std::cout << "whichStance: " << whichStance << std::endl;
    // std::cout << "phaseVar: " << phaseVar << std::endl;

	vector_t y_des = vector_t::Zero(12);
	bezier_tools::bezier(coeff_cur,phaseVar,y_des);
	vector_t dy_des = vector_t::Zero(12);
	bezier_tools::dbezier(coeff_cur,phaseVar,dy_des);
    dy_des = dy_des * (1/walkStepDur_); 


  return std::tie(y_des,dy_des);
}

void walking::evalActualTaskSpaceState(Exo_t::vector_t& y_act,Exo_t::vector_t& yh, Exo_t::vector_t& dy_act, const mjModel* model, const mjData* data,int whichStance) {
  // Get the actual CoM position
  // double* com = SensorByName(model, data, "torso_subcom");
  // y_act.segment(0,3) << com[0], com[1], com[2];
  mjData* data_ = const_cast<mjData*>(data);
//   mj_comPos(model, data_);
//   mj_comVel(model, data_);
  double com[3] = {0.0};
  mju_addTo(com, data_->subtree_com, 3);
  y_act.segment(0,3) << com[0], com[1], com[2];

  // std::cout << "com " << y_act.segment(0,3).transpose() << std::endl;

  // Get the actual CoM velocity
  // double com_vel[3] = {0.0};
  // mju_addTo(com_vel, data_->cvel, 3);
  double* com_vel = SensorByName(model, data, "torso_subcomvel");
  dy_act.segment(0,3) << com_vel[0], com_vel[1], com_vel[2];

  //Get Pelvis Orientation
  vector_t quat_act = vector_t::Zero(4);
  quat_act << data->qpos[3],data->qpos[4],data->qpos[5],data->qpos[6];
  vector_t pelv_act = utils::quat2eulXYZ(quat_act);
  
  //Get Pelvis Angular Velocity
  // vector_t omega = vector_t::Zero(3);
  // omega << data->qvel[3],data->qvel[4],data->qvel[5];
  // dy_act.segment(3,3) = utils::omega2eulRates(pelv_act,omega);
  double * pelv_ang_vel = SensorByName(model, data, "torso_angvel");
  dy_act.segment(3,3) << pelv_ang_vel[0], pelv_ang_vel[1],pelv_ang_vel[2];

  double* foot_right = SensorByName(model, data, "right_heel");
  double* foot_left = SensorByName(model, data, "left_heel");
  double* foot_left_vel = SensorByName(model, data, "foot_left_vel");
  double* foot_right_vel = SensorByName(model, data, "foot_right_vel");
  double* foot_right_ang_vel = SensorByName(model, data, "foot_right_ang");
  double* foot_left_ang_vel = SensorByName(model, data, "foot_left_ang");

  double* foot_right_ori = SensorByName(model, data, "right_heel_ori");
  // convert to eul
  vector_t right_foot_quat = vector_t::Zero(4);
  right_foot_quat << foot_right_ori[0], foot_right_ori[1],foot_right_ori[2],foot_right_ori[3];
  vector_t right_foot_ori = utils::quat2eulXYZ(right_foot_quat);

  //repeat for left foot
  double* foot_left_ori = SensorByName(model, data, "left_heel_ori");
  // convert to eul
  vector_t left_foot_quat = vector_t::Zero(4);
  left_foot_quat << foot_left_ori[0], foot_left_ori[1],foot_left_ori[2],foot_left_ori[3];
  vector_t left_foot_ori = utils::quat2eulXYZ(left_foot_quat);

  //get angmom
  // double* com_angmom = SensorByName(model, data, "torso_subangmom");

  //get foot_right_up and foot_left_up
  double* foot_right_up = SensorByName(model, data, "foot_right_up");
  double* foot_left_up = SensorByName(model, data, "foot_left_up");

  vector_t stance_foot_pos = vector_t::Zero(3);
  vector_t swing_foot_pos = vector_t::Zero(3);
  vector_t swing_foot_ori = vector_t::Zero(3);

  switch(whichStance){
    case Left:{
      // get stance and swing foot pos
      swing_foot_pos << foot_right[0], foot_right[1],foot_right[2];
      stance_foot_pos << foot_left[0], foot_left[1],foot_left[2];

      swing_foot_ori = right_foot_ori;
      dy_act.segment(6,3) << foot_right_vel[0], foot_right_vel[1],foot_right_vel[2];
      dy_act.segment(9,3) << foot_right_ang_vel[0], foot_right_ang_vel[1],foot_right_ang_vel[2];      

      yh.segment(0,3) << foot_left_up[0], foot_left_up[1],foot_left_up[2]-1;
   
      break;}
    case Right:{
      // get stance and swing foot pos
      swing_foot_pos << foot_left[0], foot_left[1],foot_left[2];
      stance_foot_pos << foot_right[0], foot_right[1],foot_right[2];

      swing_foot_ori = left_foot_ori;
      dy_act.segment(6,3) << foot_left_vel[0], foot_left_vel[1],foot_left_vel[2];
      dy_act.segment(9,3) << foot_left_ang_vel[0], foot_left_ang_vel[1],foot_left_ang_vel[2];

      yh.segment(0,3) << foot_right_up[0], foot_right_up[1],foot_right_up[2]-1;
    
      break;}
  }

  double* torso_up = SensorByName(model, data, "torso_up");
  yh.segment(3,3) << torso_up[0], torso_up[1],torso_up[2]-1;
  y_act.segment(0,3) = y_act.segment(0,3) - stance_foot_pos;
  y_act.segment(3,3) = pelv_act;
  y_act.segment(6,3) = swing_foot_pos-stance_foot_pos;
  y_act.segment(9,3) = swing_foot_ori;

}

void walking::ResidualFn::Residual(const mjModel* model, const mjData* data,
                                 double* residual) const {

  
     // Calculate the number of complete cycles
    int whichStance = data->userdata[0];
    double initial_t0 = static_cast<double>(data->userdata[1]);
    double current_time = data->time - initial_t0;
    //  scalar_t phaseVar = (data->time - initial_t0) / walkStepDur;  
    vector_t q_des,dq_des;
    std::tie(q_des,dq_des) = walking::evalJtBezier(current_time,coeff,coeff_remap,whichStance,walkStepDur);



    int counter = 0;
    //convert desire quat to euler
    vector_t q_des_euler = utils::quat2eulXYZ(q_des.segment(3,4));
    //convert actual quat to euler
    vector_t quat_act = vector_t::Zero(4);
    quat_act << data->qpos[3],data->qpos[4],data->qpos[5],data->qpos[6];
    vector_t q_act = utils::quat2eulXYZ(quat_act);



    //get desired task space trajectory
    vector_t y_des,dy_des;
    std::tie(y_des,dy_des) = walking::evalTaskBezier(current_time,coeff_task,coeff_task_remap,whichStance,walkStepDur);

    //get actual task space trajectory, calculate holonomic constraint violation --> velocity
    vector_t y_act = vector_t::Zero(12);
    vector_t dy_act = vector_t::Zero(12);
    vector_t y_h = vector_t::Zero(6);
    //eval com2st, pelv_ori, sw_ft, sw_ft_ori
    walking::evalActualTaskSpaceState(y_act,y_h,dy_act,model,data,whichStance);

    //update residual task space
    for(int i=0; i< 12;i++){
    residual[counter++] = (y_des[i] - y_act[i]);
    }

    for(int i=0; i< 12;i++){

    residual[counter++] = (dy_des[i] - dy_act[i]);

    }
    //TODO: update to be actual holonomic constraint
    for(int i=0; i< 6;i++){
    if(i<3){
        residual[counter++] = y_h[i];
    }
    else{
        residual[counter++] = y_h[i];
    }
    }



    //calculate joint err
    for(int i=0; i< 12;i++){
    residual[counter++] = (q_des[i+7] - data->qpos[i+7]);
    }

    for(int i=0; i< 12;i++){
    residual[counter++] = (dq_des[i+6] - data->qvel[i+6]);
    }


    int user_sensor_dim = 0;
    for (int i = 0; i < model->nsensor; i++) {
        if (model->sensor_type[i] == mjSENS_USER) {
        user_sensor_dim += model->sensor_dim[i];
        }
    }
    if (user_sensor_dim != counter) {
        mju_error_i(
            "mismatch between total user-sensor dimension "
            "and actual length of residual %d",
            counter);
    }



}


void walking::TransitionLocked(mjModel* model, mjData* data) {
  // If falling update based on current policy
  // mjData* data_ = const_cast<mjData*>(data);
  mj_comPos(model, data);
  double com[3] = {0.0};
  mju_addTo(com, data->subtree_com, 3);
  if(com[2]<0.75){
    int home_id = mj_name2id(model, mjOBJ_KEY, "home");
    if (home_id >= 0) mj_resetDataKeyframe(model, data, home_id);
    initial_t0 = data->time;
    whichStance = Right;
  }

  //check grf
  vector_t grf = vector_t::Zero(8);
  for(int i=0; i< 8;i++){
    grf[i] = *SensorByName(model, data, "opto" + std::to_string(i+1));
  }
//   std::cout << "grf " << grf.transpose() << std::endl;

  double left_grf = grf.segment(0,4).sum();
  double right_grf = grf.segment(4,4).sum();

  switch (whichStance){
    case Left:{
      if (right_grf > 400 && data->time - initial_t0 > 0.5){
        whichStance = Right;
        initial_t0 = data->time;
        std::cout << "Switch to Right" << std::endl;
      }
      break;
    }
    case Right:{
      if (left_grf > 400 && data->time - initial_t0 > 0.5){
        whichStance = Left;
        initial_t0 = data->time;
        std::cout << "Switch to Left" << std::endl;
      }
      break;
    }
  }
//   std::cout << model->nuserdata << std::endl; 
  data->userdata[0] = whichStance;
  data->userdata[1] = initial_t0;
}

}  // namespace mjpc::exo
