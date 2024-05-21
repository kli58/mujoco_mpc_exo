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

#ifndef MJPC_TASKS_EXO_WALK_TASK_H_
#define MJPC_TASKS_EXO_WALK_TASK_H_

#include <memory>
#include <string>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/tasks/exo/Types.h"
#include "mjpc/tasks/exo/bezier_tools.hpp"


namespace mjpc {
namespace exo {

class walking : public Task {
 public:
  class ResidualFn : public mjpc::BaseResidualFn {
   public:

    void loadGoalJtPosition();
    
    //bez polynomial representation
    Exo_t::matrix_t coeff;
    Exo_t::matrix_t coeff_remap;
    Exo_t::matrix_t coeff_task;
    Exo_t::matrix_t coeff_task_remap;

    Exo_t::scalar_t walkStepDur;
    
    explicit ResidualFn(const walking* task) : mjpc::BaseResidualFn(task) {loadGoalJtPosition();}

    void Residual(const mjModel* model, const mjData* data,
                  double* residual) const override;

    
  };

  void loadGoalJtPosition();
  std::tuple<int,double> checkGRF(mjModel* model, mjData* data);
  void TransitionLocked(mjModel* model, mjData* data) override;
  static void evalActualTaskSpaceState(Exo_t::vector_t& y_act,Exo_t::vector_t& yh, Exo_t::vector_t& dy_act, const mjModel* model, const mjData* data,int whichStance);
  walking() : residual_(this) {loadGoalJtPosition();};
  std::vector<double> des;
  
  void convertAction(double phaseVar, int curStance,double* action, double* task_space_action, const mjModel* model,mjData* kin_data) const;
  void GetNominalPlanAction(const mjModel* model, double* action,double* task_space_action, mjData* kin_data, double time,double* userData) const override;
  void GetNominalAction(const mjModel* model, double* action,double* task_space_action, mjData* kin_data, double time) const override;
  void UpdateUserData(const mjModel* model, mjData* data) const override;
  std::string Name() const override;
  std::string XmlPath() const override;


  Exo_t::matrix_t coeff;
  Exo_t::matrix_t coeff_remap;
  Exo_t::matrix_t coeff_task;
  Exo_t::matrix_t coeff_task_remap;

  mjData* kin_data;
  Exo_t::scalar_t walkStepDur;
  static std::tuple<Exo_t::vector_t,Exo_t::vector_t>  evalJtBezier(double time, Exo_t::matrix_t coeff_, Exo_t::matrix_t coeff_remap_,int whichStance,Exo_t::scalar_t walkStepDur_);
  static std::tuple<Exo_t::vector_t,Exo_t::vector_t>  evalTaskBezier(double time, Exo_t::matrix_t coeff_, Exo_t::matrix_t coeff_remap_,int whichStance,Exo_t::scalar_t walkStepDur_);
  

 protected:
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<ResidualFn>(this);
  }
  ResidualFn* InternalResidual() override { return &residual_; }

 private:
  ResidualFn residual_;
};

}  // namespace exo
}  // namespace mjpc

#endif  // MJPC_TASKS_EXO_TS_WALK_TASK_H_
