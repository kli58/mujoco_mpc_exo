#include "mjpc/tasks/exo/stair_walking.h"
#include "mjpc/tasks/exo/Types.h"
#include "mjpc/tasks/exo/utils.hpp"
#include <string>

#include <mujoco/mujoco.h>
#include <absl/random/random.h>

#include "yaml-cpp/yaml.h"
#include <algorithm>
#include "mjpc/tasks/exo/ExoConstants.hpp"

namespace mjpc::exo {

using namespace Exo_t;


void stair_walking::GetNominalPlanAction(const mjModel* model, double* action, double* task_space_action, mjData* kin_data, double time,double* userData) const {
       //instead of use the qpos from state, use the qpos from evaluate jt bezier
    // int numCycle = static_cast<int>((time) / walkStepDur);

    int curStance = userData[0];
    
    double phaseVar = (time - userData[1])/walkStepDur;
    
    phaseVar = std::min(1.0, std::max(0.0, phaseVar));
  //  std::cout << "nomianl plan action " << phaseVar << " " << curStance << " " << time << " " << userData[1] << " " << walkStepDur << std::endl;
   convertAction(phaseVar,curStance,action,task_space_action,model,kin_data);

}

void stair_walking::GetNominalAction(const mjModel* model, double* action, double* task_space_action, mjData* kin_data,double time) const {
                              // std::cout << "Getting default Walkinging pos" << std::endl;


  //instead of use the qpos from state, use the qpos from evaluate jt bezier
  int curStance = whichStance;
    
  double phaseVar = (time - initial_t0)/walkStepDur;
  phaseVar = std::min(1.0, std::max(0.0, phaseVar));
  convertAction(phaseVar,curStance,action,task_space_action, model,kin_data);
}


void stair_walking::convertAction(double phaseVar, int curStance, double* action, double* task_space_action, const mjModel* model,mjData* kin_data) const{
  vector_t q_init,dq_init;
    std::tie(q_init,dq_init) = stair_walking::evalJtBezier(phaseVar,coeff,coeff_remap,curStance, walkStepDur);

    bool jt_space = false;
    if(jt_space){
        for (int i = 0; i < 12; i++) {
            action[i] = scale[i] *task_space_action[i] + q_init[i+7];
        }

        for(int i=0; i<12;i++){
          action[i+12] = dq_init[i+6];
            // action[i+12] = scale[i] *task_space_action[i+12] + dq_init[i+6];
        }
    }
    else{
        // std::cout << "IK mode" << std::endl;

        matrix_t J_y = matrix_t::Zero(12,18);
        matrix_t J_h = matrix_t::Zero(6,18);


        Eigen::VectorXd yDesFull = Eigen::VectorXd::Zero(18);

        //loop through the first 12 element
        vector_t yd,dyd = vector_t::Zero(12);
        std::tie(yd,dyd) = evalTaskBezier(phaseVar,coeff_task,coeff_task_remap,curStance, walkStepDur);

        // yd[2] = yd[2] - 0.005;
        yDesFull.segment(0,12) = yd;
        dyd.segment(0,12) = dyd;
    
       
        // for(int i=0; i<12;i++){
        //     yDesFull[i] = yd[i] + scale[i]*action[i];
        //     dyd[i] = dyd[i] + scale[i]*action[i+12];
        // }
        // std::cout << "yd: " << yd.transpose() << std::endl;
        // std::cout << "dyd: " << dyd.transpose() << std::endl;
        //loop through action dim and print out task space_action
        // std::cout << "task_space_action: ";
        // for(int i=0; i<action_dim*2;i++){
        //     std::cout << task_space_action[i] << " ";
        // }
        // std::cout << std::endl;
        
        // std::cout << "scale: ";
        // for(int i=0; i<12;i++){
        //     std::cout << scale[i] << " ";
        // }
        // std::cout << std::endl;

        // check is task spac is nan
        for(int i=0; i<action_dim;i++){
            if(std::isnan(task_space_action[i])){
                std::cout << "task space action is nan" << std::endl;
                std::terminate();
            }
        }

        switch(action_dim){
          case 12:{
            for(int i=0; i<12;i++){
              
                yDesFull[i] = yd[i] + scale[i]*task_space_action[i];
                dyd[i] = dyd[i] + scale[i]*task_space_action[i+12];
            }
            
          
      
      
            break;
          }

          case 3:{ //com position only
            for(int i=0; i<3;i++){
                yDesFull[i] = yd[i] + scale[i]*task_space_action[i];
                //bounded the value based on action bound, order is lb_0,ub_0, lb_1, ub_1
                // yDesFull[i] = fmin(fmax(yDesFull[i],action_bound[i*2]),action_bound[i*2+1]);
            }
           
            break;
            
          }

            case 6:{ //com position and pevlis orientation
            for(int i=0; i<6;i++){
                yDesFull[i] = yd[i] + scale[i]*task_space_action[i];
                dyd[i] = dyd[i] + scale[i]*task_space_action[i+6];
                //bounded the value based on action bound, order is lb_0,ub_0, lb_1, ub_1
                // yDesFull[i] = fmin(fmax(yDesFull[i],action_bound[i*2]),action_bound[i*2+1]);
            }
           
            break;
            
          }
          
          case 9:{ //com pos, pelvis ori, sw pos
            for(int i=0; i<9;i++){
                yDesFull[i] = yd[i] + scale[i]*task_space_action[i];
                dyd[i] = dyd[i] + scale[i]*task_space_action[i+9];
            }
         
            break;
          }
     
        }


        Eigen::VectorXd err;
        bool success = false;
        int maxIKIterations = 10;
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
        bool use_q_init = false;
        for (int i = 0; i < maxIKIterations; ++i) {
            //print qpos
            
            mj_kinematics(model, kin_data);
            mj_comPos(model, kin_data);


            taskspace_utils::getStairY(yCur,model,kin_data, curStance);

            err = yDesFull - yCur;
          

            if (err.cwiseAbs().maxCoeff() < 1e-3) {
                success = true;
                // std::cout << "IK converged in " << i << " iterations" << std::endl;
                break;
            }

            taskspace_utils::getStairJacobian(J,model,kin_data, curStance);
            Eigen::MatrixXd JJt = J * J.transpose() + Eigen::MatrixXd::Identity(J.rows(), J.rows()) * dampingFactor;
            Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse(); // Compute the pseudo-inverse using COD
            Eigen::VectorXd deltaQ = J_pinv * err; // Directly use the pseudo-inverse to compute deltaQ
    
            

            for (int j = 0; j < model->nv; j++) {
                kin_data->qvel[j] = deltaQ[j];
            }

            mj_integratePos(model, kin_data->qpos, kin_data->qvel, 1);
        }

        if (!success) {
            std::cout << "IK did not converge" << std::endl;
            std::cout << "IK yd: " << yd.transpose() << std::endl;
            use_q_init = true;
        }

  
        matrix_t J_output = J.block(0,0,12,18);
        vector_t dq_ik = J_output.transpose()*(J_output * J_output.transpose()).inverse() * dyd;
        

        if (use_q_init) {
              // overide action with current qpos
          for (int i = 0; i < 12; i++) {
            action[i] = q_init[i+7];
          }

          for(int i=0; i<12;i++){
            action[i+12] = dq_init[i+6];
          }
        }
        else{
          // overide action with current qpos
          for (int i = 0; i < 12; i++) {
            action[i] = kin_data->qpos[i+7];
          }

          for(int i=0; i<12;i++){
            action[i+12] = dq_ik[i+6];
          }
        }
    }
}

std::string stair_walking::XmlPath() const {
  return GetModelPath("exo/plan_task.xml");
}


std::string stair_walking::Name() const { return "Exo stair walking"; }

void stair_walking::ResidualFn::loadGoalJtPosition(){
  std::string FilePath = GetModelPath("exo/default_stair_bez.yaml");
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

void stair_walking::loadGoalJtPosition(){
  std::string FilePath = GetModelPath("exo/default_stair_bez.yaml");
	std::string TaskSpeification = GetModelPath("exo/task_specification.yaml");
	YAML::Node config = YAML::LoadFile(FilePath);
  YAML::Node task_spec = YAML::LoadFile(TaskSpeification);
  
  //whether to use nominal policy or not
  nominal_policy = task_spec["nominal_policy"].as<bool>();
  traj_perturb = task_spec["traj_perturb"].as<bool>();
  scale = task_spec["action_scale"].as<std::vector<double>>();
  
  uint_fast32_t random_seed = task_spec["random_seed"].as<uint_fast32_t>();
  gen.seed(random_seed);

  terminate_early = task_spec["terminate_early"].as<bool>();

  if(nominal_policy){
    //overwrite scale to be all zero
    scale = std::vector<double>(12,0);
  }
  //start logger
  std::string dataLog = task_spec["log_file"].as<std::string>();
  fileHandle.open(dataLog);

  std::string paramLog = task_spec["CE_param_file"].as<std::string>();
  CEParamsHandle.open(paramLog);

  //load perturbation and sampler param
  xfrc_rate = task_spec["xfrc_rate"].as<double>();
  xfrc_std = task_spec["xfrc_std"].as<double>();
  xfrc_mean = task_spec["xfrc_mean"].as<double>();
  perturb_body = task_spec["perturb_body"].as<std::vector<double>>();

  action_dim = task_spec["action_dim"].as<int>();
  action_bound = new double[action_dim*2];
  std::vector<double> com_bound = task_spec["com_bound"].as<std::vector<double>>();
  std::vector<double> sw_ft_bound = task_spec["sw_ft_bound"].as<std::vector<double>>();
  std::vector<double> pelvis_bound = task_spec["pelvis_bound"].as<std::vector<double>>();
  std::vector<double> sw_ft_ori_bound = task_spec["sw_ft_ori_bound"].as<std::vector<double>>();
  
  switch(action_dim){

    case 3:{
      
      for(int i=0; i<action_dim;i++){
        action_bound[2*i] = com_bound[2*i];
        action_bound[2*i+1] = com_bound[2*i+1];
      }

      for(int i=0; i<action_dim;i++){
        action_bound[2*i+6] = -1;
        action_bound[2*i+7] = 1;
      }

      break;
    }//com position only
    case 12:{
      
      std::vector<double> all_bounds;
      all_bounds.insert(all_bounds.end(), com_bound.begin(), com_bound.end());
      all_bounds.insert(all_bounds.end(), pelvis_bound.begin(), pelvis_bound.end());
      all_bounds.insert(all_bounds.end(), sw_ft_bound.begin(), sw_ft_bound.end());
      all_bounds.insert(all_bounds.end(), sw_ft_ori_bound.begin(), sw_ft_ori_bound.end());

      // Populate action_bound with the concatenated bounds
      int bound_size = all_bounds.size();
      for (int i = 0; i < bound_size; i++) {
          action_bound[i] = all_bounds[i];
      }
    break;
      }//com position and com vel; swing position and vel;

    case 9:{
      std::vector<double> all_bounds;
      all_bounds.insert(all_bounds.end(), com_bound.begin(), com_bound.end());
      all_bounds.insert(all_bounds.end(), pelvis_bound.begin(), pelvis_bound.end());
      all_bounds.insert(all_bounds.end(), sw_ft_bound.begin(), sw_ft_bound.end());

      // Populate action_bound with the concatenated bounds
      int bound_size = all_bounds.size();
      for (int i = 0; i < bound_size; i++) {
          action_bound[i] = all_bounds[i];
      }

      break;}

      case 6:{
      std::vector<double> all_bounds;
      all_bounds.insert(all_bounds.end(), com_bound.begin(), com_bound.end());
      all_bounds.insert(all_bounds.end(), pelvis_bound.begin(), pelvis_bound.end());

      // Populate action_bound with the concatenated bounds
      int bound_size = all_bounds.size();
      for (int i = 0; i < bound_size; i++) {
          action_bound[i] = all_bounds[i];
      }

      break;}
  }
  
 
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



std::tuple<vector_t,vector_t> stair_walking::evalJtBezier(double time,matrix_t coeff_, matrix_t coeff_remap_,int stance, scalar_t walkStepDur_) {

  // Calculate the phase variable within the current cycle
  scalar_t phaseVar = (time) / walkStepDur_;

	matrix_t coeff_cur = matrix_t::Zero(18,8);
	switch(stance){
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

std::tuple<vector_t,vector_t> stair_walking::evalTaskBezier(double time,matrix_t coeff_, matrix_t coeff_remap_,int stance, scalar_t walkStepDur_){
  // Get the phase variable

    // Calculate the number of complete cycles
//   int numCycle = static_cast<int>(time / walkStepDur_);

  // Calculate the phase variable within the current cycle
  scalar_t phaseVar = (time) / walkStepDur_;


	matrix_t coeff_cur = matrix_t::Zero(12,8);
	switch(stance){
		case Left:{//left stance; frost assume right stance so need to use the remap coefficient
			coeff_cur  = coeff_remap_; 
            // std::cout  << "left stance" << std::endl;
			break;}
		case Right:{
			coeff_cur = coeff_;
            // std::cout << "right stance" << std::endl;
			break;}
	}
    

	vector_t y_des = vector_t::Zero(12);
	bezier_tools::bezier(coeff_cur,phaseVar,y_des);
	vector_t dy_des = vector_t::Zero(12);
	bezier_tools::dbezier(coeff_cur,phaseVar,dy_des);
    dy_des = dy_des * (1/walkStepDur_); 


  return std::tie(y_des,dy_des);
}

void stair_walking::evalActualTaskSpaceState(Exo_t::vector_t& y_act,Exo_t::vector_t& yh, Exo_t::vector_t& dy_act, const mjModel* model, const mjData* data,int whichStance) {
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

  double* foot_right = SensorByName(model, data, "right_sole");
  double* foot_left = SensorByName(model, data, "left_sole");

  double* foot_right_toe = SensorByName(model, data, "right_toe");
  double* foot_left_toe = SensorByName(model, data, "left_toe");

  double* foot_left_vel = SensorByName(model, data, "foot_left_vel");
  double* foot_right_vel = SensorByName(model, data, "foot_right_vel");
  double* foot_right_ang_vel = SensorByName(model, data, "foot_right_ang");
  double* foot_left_ang_vel = SensorByName(model, data, "foot_left_ang");

  double* foot_right_ori = SensorByName(model, data, "right_sole_ori");
  // convert to eul
  vector_t right_foot_quat = vector_t::Zero(4);
  right_foot_quat << foot_right_ori[0], foot_right_ori[1],foot_right_ori[2],foot_right_ori[3];
  vector_t right_foot_ori = utils::quat2eulXYZ(right_foot_quat);

  //repeat for left foot
  double* foot_left_ori = SensorByName(model, data, "left_sole_ori");
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

  switch(int(data->userdata[0])){
    case Left:{
      // get stance and swing foot pos
      swing_foot_pos << foot_right_toe[0], foot_right_toe[1],foot_right_toe[2];
      stance_foot_pos << foot_left[0], foot_left[1],foot_left[2];

      swing_foot_ori = right_foot_ori;
      dy_act.segment(6,3) << foot_right_vel[0], foot_right_vel[1],foot_right_vel[2];
      dy_act.segment(9,3) << foot_right_ang_vel[0], foot_right_ang_vel[1],foot_right_ang_vel[2];      

      yh.segment(0,3) << foot_left_up[0], foot_left_up[1],foot_left_up[2]-1;
   
      break;}
    case Right:{
      // get stance and swing foot pos
      swing_foot_pos << foot_left_toe[0], foot_left_toe[1],foot_left_toe[2];
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

void stair_walking::ResidualFn::Residual(const mjModel* model, const mjData* data,
                                 double* residual) const {

  
     // Calculate the number of complete cycles
    int stance = data->userdata[0];
    double initial_t0 = static_cast<double>(data->userdata[1]);
    double current_time = data->time - initial_t0;
    //  scalar_t phaseVar = (data->time - initial_t0) / walkStepDur;  
    vector_t q_des,dq_des;
    std::tie(q_des,dq_des) = stair_walking::evalJtBezier(current_time,coeff,coeff_remap,stance,walkStepDur);



    int counter = 0;
    //convert desire quat to euler
    vector_t q_des_euler = utils::quat2eulXYZ(q_des.segment(3,4));
    //convert actual quat to euler
    vector_t quat_act = vector_t::Zero(4);
    quat_act << data->qpos[3],data->qpos[4],data->qpos[5],data->qpos[6];
    vector_t q_act = utils::quat2eulXYZ(quat_act);



    //get desired task space trajectory
    vector_t y_des,dy_des;
    std::tie(y_des,dy_des) = stair_walking::evalTaskBezier(current_time,coeff_task,coeff_task_remap,stance,walkStepDur);

    //get actual task space trajectory, calculate holonomic constraint violation --> velocity
    vector_t y_act = vector_t::Zero(12);
    vector_t dy_act = vector_t::Zero(12);
    vector_t y_h = vector_t::Zero(6);
    //eval com2st, pelv_ori, sw_ft, sw_ft_ori
    stair_walking::evalActualTaskSpaceState(y_act,y_h,dy_act,model,data,stance);

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

void stair_walking::UpdateUserData(const mjModel* model, mjData* data) const{
  //check grf
    vector_t grf = vector_t::Zero(8);
    for(int i=0; i< 8;i++){
      grf[i] = *SensorByName(model, data, "opto" + std::to_string(i+1));
    }
    // std::cout << "planning grf " << grf.transpose() << std::endl;

    double left_grf = grf.segment(0,4).sum();
    double right_grf = grf.segment(4,4).sum();


    int curStance = data->userdata[0];
    scalar_t curStancet0 = data->userdata[1];
    switch (curStance){
      case Left:{
        if ((right_grf > 500 && data->time - curStancet0 > 0.9) || data->time-curStancet0 >= 1.0){
          curStance = Right;
          curStancet0 = data->time;
          // std::cout << "Planner Switch to Right" << std::endl;
        }
        break;
      }
      case Right:{
        if ((left_grf > 500 && data->time - curStancet0 > 0.9) || data->time-curStancet0 >= 1.0){
          curStance = Left;
          curStancet0 = data->time;
          // std::cout << "Planner Switch to Left" << std::endl;
        }
        break;
      }
    }

    data->userdata[0] = curStance;
    data->userdata[1] = curStancet0;
    // std::cout << "update planning userdata " << data->userdata[0] << " " << data->userdata[1] << std::endl;
}

void stair_walking::UpdatePolicyParam(const std::vector<double>& src_parameters,
    const std::vector<double>& src_times, int num_spline_points) const{

  // log the parameters in csv file
  // loop through num_spline_points to log src_times
  for(int i=0; i<num_spline_points;i++){
    CEParamsHandle << src_times[i] << ",";
  }
  for(int i=0; i < num_spline_points*action_dim;i++){
    CEParamsHandle << src_parameters[i] << ",";
  }
    CEParamsHandle << std::endl;
  }

void stair_walking::TransitionLocked(mjModel* model, mjData* data) {
  // If falling update based on current policy
  // mjData* data_ = const_cast<mjData*>(data);
  mj_comPos(model, data);
  double com[3] = {0.0};
  mju_addTo(com, data->subtree_com, 3);
  if(com[2]<0.75){
    if(terminate_early && data->time > 0){
      std::terminate();
    }
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
 

  double* stancefootPos;

  switch (whichStance){
    case Left:{
      stancefootPos = SensorByName(model, data, "foot_left_up");
      if (right_grf > 400 && data->time - initial_t0 > 0.8){
        whichStance = Right;
        initial_t0 = data->time;
        std::cout << "Switch to Right" << std::endl;
      }
      break;
    }
    case Right:{
      stancefootPos = SensorByName(model, data, "foot_right_up");
      if (left_grf > 400 && data->time - initial_t0 > 0.8){
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

  vector_3t force = vector_3t::Zero();
  vector_3t torque = vector_3t::Zero();
  vector_t force_norm = vector_t::Zero(perturb_body.size());
  vector_t torque_norm = vector_t::Zero(perturb_body.size());
//calculate perturbation force/torque
  //loop through perturb_body
  for(int i=0; i< perturb_body.size();i++){
     int body_id = perturb_body[i];

     force << data->xfrc_applied[6*body_id], data->xfrc_applied[6*body_id+1], data->xfrc_applied[6*body_id+2];
     torque << data->xfrc_applied[6*body_id+3], data->xfrc_applied[6*body_id+4], data->xfrc_applied[6*body_id+5];
     force_norm[i] = force.norm();
      torque_norm[i] = torque.norm();
  }

  //loop through fields to log: sim_time, stance, initial_t0, com_pos
  if (data->time > 0.0){
    fileHandle << data->time << "," 
              << data->userdata[0] << "," 
              << data->userdata[1] << "," 
              << data->subtree_com[0]<< "," 
              << data->subtree_com[1]<< "," 
              << data->subtree_com[2] << ","
              << stancefootPos[0] << ","
              << stancefootPos[1] << ","
              << stancefootPos[2] << ","
              << force_norm[0] << ","
              << force_norm[1] << ","
              << torque_norm[0] << ","
              << torque_norm[1]
              << std::endl;
  }

  if(data->time > 15){
    // std::terminate();
  }
}

}  // namespace mjpc::exo
