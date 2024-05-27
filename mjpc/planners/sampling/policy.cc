// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mjpc/planners/sampling/policy.h"

#include <algorithm>
#include <vector>

#include <mujoco/mujoco.h>
#include "mjpc/planners/policy.h"
#include "mjpc/task.h"
#include "mjpc/trajectory.h"
#include "mjpc/utilities.h"

namespace mjpc {

// allocate memory
void SamplingPolicy::Allocate(const mjModel* model, const Task& task,
                              int horizon) {
  // model
  this->model = model;

  //task 
  this->task = &task;

  // kinematics_data

  this->kin_data = mj_makeData(model);

  // parameters
  parameters.resize(task.action_dim * kMaxTrajectoryHorizon);

  // task space action
  task_space_action = new double[2*task.action_dim];

  // times
  times.resize(kMaxTrajectoryHorizon);

  // dimensions
  num_parameters = task.action_dim * kMaxTrajectoryHorizon;
  // std::cout << "num_parameters: " << num_parameters << std::endl;

  // task space action
  action_dim_ = task.action_dim;


  // spline points
  num_spline_points = GetNumberOrDefault(kMaxTrajectoryHorizon, model,
                                         "sampling_spline_points");

  // representation
  representation = GetNumberOrDefault(PolicyRepresentation::kCubicSpline, model,
                                      "sampling_representation");
}

// reset memory to zeros
void SamplingPolicy::Reset(int horizon, const double* initial_repeated_action) {
  // parameters
  // if (initial_repeated_action != nullptr) {
  //   for (int i = 0; i < num_spline_points; ++i) {
  //     mju_copy(parameters.data() + i * action_dim_, initial_repeated_action,
  //              action_dim_);
  //   }
  // } else {
    //always set to zero to avoid conflict
    std::fill(parameters.begin(),
              parameters.begin() + action_dim_* num_spline_points, 0.0);
  // }

  std::fill(task_space_action, task_space_action + 2*action_dim_, 0.0);
  // policy parameter times
  std::fill(times.begin(), times.begin() + horizon, 0.0);
}

// set action from policy
void SamplingPolicy::Action(double* action, const double* state, double time) const {
  // find times bounds
  int bounds[2];
  FindInterval(bounds, times, time, num_spline_points);

  // double* task_space_action = new double[action_dim_];
  // std::cout << "bounds[0]: " << bounds[0] << " bounds[1]: " << bounds[1] << std::endl;
  
  //loop through parameters and print the values out 
  // for(int i = 0; i < num_parameters; i++){
  //   // std::cout << " parameters[" << i << "]: " << parameters[i];
  // }
  // std::cout << std::endl;

  // ----- get action ----- //
  if (bounds[0] == bounds[1] ||
      representation == PolicyRepresentation::kZeroSpline) {
    ZeroInterpolation(task_space_action, time, times, parameters.data(), action_dim_,
                      num_spline_points);
    //velocty term should be zero
    for(int i = action_dim_; i < 2*action_dim_; i++){
      task_space_action[i] = 0.0;
    }

  } else if (representation == PolicyRepresentation::kLinearSpline) {
    LinearInterpolation(task_space_action, time, times, parameters.data(), action_dim_,
                        num_spline_points);
  } else if (representation == PolicyRepresentation::kCubicSpline) {
    CubicInterpolationWithVelocity(task_space_action, time, times, parameters.data(), action_dim_,
                       num_spline_points);

    
  }

    // for(int i = 0; i < action_dim_*2; i++){
    // // if(std::isnan(task_space_action[i])){
    //   std::cout << "Action task_space_action[" << i << "]: " << task_space_action[i] << std::endl;
    //   // std::terminate();
    // // }
    //  }

  // convert nominal task space action to joint space
  task->GetNominalAction(model,action,task_space_action, kin_data, time);

  // delete[] task_space_action;
  // Clamp controls
  Clamp(action, model->actuator_ctrlrange, model->nu);
}


void SamplingPolicy::TaskSpaceAction(double* task_action, const double* state, double time) const {
  // find times bounds
  int bounds[2];
  FindInterval(bounds, times, time, num_spline_points);

  // ----- get action ----- //
  if (bounds[0] == bounds[1] ||
      representation == PolicyRepresentation::kZeroSpline) {
    ZeroInterpolation(task_action, time, times, parameters.data(), action_dim_,
                      num_spline_points);
    //velocty term should be zero
    for(int i = action_dim_; i < 2*action_dim_; i++){
      task_space_action[i] = 0.0;
    }
  } else if (representation == PolicyRepresentation::kLinearSpline) {
    LinearInterpolation(task_action, time, times, parameters.data(), action_dim_,
                        num_spline_points);
  } else if (representation == PolicyRepresentation::kCubicSpline) {
    CubicInterpolationWithVelocity(task_space_action, time, times, parameters.data(), action_dim_,
                       num_spline_points);
  }

  // std::cout << "TaskSpaceAction task_space_action" ;
  //   for(int i = 0; i < action_dim_*2; i++){
  //   // if(std::isnan(task_space_action[i])){
  //    std::cout  << " " << task_space_action[i]; 
  //     // std::terminate();
  //   // }
  // }
  // std::cout << std::endl;

}

void SamplingPolicy::Plan_Action(double* action, mjData* plan_kin_data, const double* state, double time, double* userData) const {
  // find times bounds
  int bounds[2];
  FindInterval(bounds, times, time, num_spline_points);

  // ----- get action ----- //
  // double* task_space_action = new double[action_dim_];
  // std::cout << "Plan bounds[0]: " << bounds[0] << " bounds[1]: " << bounds[1] << std::endl;
  // std::cout << "parameters.data() " << parameters.data() << std::endl;
  if (bounds[0] == bounds[1] ||
      representation == PolicyRepresentation::kZeroSpline) {
    ZeroInterpolation(task_space_action, time, times, parameters.data(), action_dim_,
                      num_spline_points);
    //velocty term should be zero
    for(int i = action_dim_; i < 2*action_dim_; i++){
      task_space_action[i] = 0.0;
    }
  } else if (representation == PolicyRepresentation::kLinearSpline) {
    LinearInterpolation(task_space_action, time, times, parameters.data(), action_dim_,
                        num_spline_points);
  } else if (representation == PolicyRepresentation::kCubicSpline) {
     CubicInterpolationWithVelocity(task_space_action, time, times, parameters.data(), action_dim_,
                       num_spline_points);
  }
  
  //check if task space action is nan
  //  std::cout << "Plan Action task_space_action" ;
  //   for(int i = 0; i < action_dim_*2; i++){
  //   // if(std::isnan(task_space_action[i])){
  //    std::cout  << " " << task_space_action[i]; 
  //     // std::terminate();
  //   // }
  // }
  // std::cout << std::endl;

  // Clamp(task_space_action, task->action_bound, action_dim_);



  // convert task space action to joint space
  task->GetNominalPlanAction(model,action,task_space_action, plan_kin_data, time,userData);


  // Clamp controls
  Clamp(action, model->actuator_ctrlrange, model->nu);
  // delete[] task_space_action;
}



// copy policy
void SamplingPolicy::CopyFrom(const SamplingPolicy& policy, int horizon) {
  mju_copy(parameters.data(), policy.parameters.data(), policy.num_parameters);
  mju_copy(times.data(), policy.times.data(), policy.num_spline_points);
  num_spline_points = policy.num_spline_points;
  num_parameters = policy.num_parameters;
}

// copy parameters
void SamplingPolicy::CopyParametersFrom(
    const std::vector<double>& src_parameters,
    const std::vector<double>& src_times) {
  mju_copy(parameters.data(), src_parameters.data(),
           num_spline_points * action_dim_);
  mju_copy(times.data(), src_times.data(), num_spline_points);
}

}  // namespace mjpc
