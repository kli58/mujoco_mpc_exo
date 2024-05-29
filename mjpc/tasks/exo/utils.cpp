#include "utils.hpp"
#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
// #include "engine/engine_core_constraint.h"
// #include "engine/engine_crossplatform.h"
// #include "engine/engine_io.h"
// #include "engine/engine_macro.h"
// #include "engine/engine_support.h"
// #include "engine/engine_util_blas.h"
// #include "engine/engine_util_errmem.h"
// #include "engine/engine_util_misc.h"
// #include "engine/engine_util_sparse.h"
// #include "engine/engine_util_spatial.h"

namespace taskspace_utils{

    void getStairY(vector_t& y_act,const mjModel* model, mjData* data,int whichStance) {
        // Get the actual CoM position
        // double* com = SensorByName(model, data, "torso_subcom");
        // y_act.segment(0,3) << com[0], com[1], com[2];
        // mj_comPos(model, data);

        double com[3] = {0.0};
        mju_addTo(com, data->subtree_com, 3);
        y_act.segment(0,3) << com[0], com[1], com[2];

        // std::cout << "com pos: " << com[0] << " " << com[1] << " " << com[2] << std::endl;

        //Get Pelvis Orientation
        vector_t quat_act = vector_t::Zero(4);
        quat_act << data->qpos[3],data->qpos[4],data->qpos[5],data->qpos[6];
        vector_t pelv_act = utils::quat2eulXYZ(quat_act);
        
        //Get Pelvis Angular Velocity
        int left_foot_id = mj_name2id(model, mjOBJ_SITE, "left_sole_frame");
        int right_foot_id = mj_name2id(model, mjOBJ_SITE, "right_sole_frame");
        int left_foot_toe_id = mj_name2id(model, mjOBJ_SITE, "left_toe_frame");
        int right_foot_toe_id = mj_name2id(model, mjOBJ_SITE, "right_toe_frame");


        //extract site pos and ori from data.site_xpos and data.site_xmat
        vector_t left_foot_pos = vector_t::Zero(3);
        vector_t right_foot_pos = vector_t::Zero(3);
        vector_t left_foot_toe_pos = vector_t::Zero(3);
        vector_t right_foot_toe_pos = vector_t::Zero(3);
        vector_t left_foot_ori = vector_t::Zero(3);
        vector_t right_foot_ori = vector_t::Zero(3);

        for(int i=0; i<3; i++){
            left_foot_pos[i] = data->site_xpos[left_foot_id*3+i];
            right_foot_pos[i] = data->site_xpos[right_foot_id*3+i];
            left_foot_toe_pos[i] = data->site_xpos[left_foot_toe_id*3+i];
            right_foot_toe_pos[i] = data->site_xpos[right_foot_toe_id*3+i];
        }

        //convert to rotation matrix to euler
        matrix_t left_foot_rot = matrix_t::Zero(3,3);
        matrix_t right_foot_rot = matrix_t::Zero(3,3);
        for(int i=0; i<3; i++){
            for(int j=0; j<3; j++){
                left_foot_rot(i,j) = data->site_xmat[left_foot_id*9+i*3+j];
                right_foot_rot(i,j) = data->site_xmat[right_foot_id*9+i*3+j];
            }
        }

        left_foot_ori = utils::quat2eulXYZ(utils::quatFrom3x3(left_foot_rot));
        right_foot_ori = utils::quat2eulXYZ(utils::quatFrom3x3(right_foot_rot));

        vector_t stance_foot_pos = vector_t::Zero(3);
        vector_t swing_foot_pos = vector_t::Zero(3);
        vector_t swing_foot_ori = vector_t::Zero(3);
        vector_t stance_foot_ori = vector_t::Zero(3);

        switch(whichStance){
            case Left:{
            // get stance and swing foot pos
            swing_foot_pos << right_foot_toe_pos;
            stance_foot_pos << left_foot_pos;

            swing_foot_ori = right_foot_ori;
            stance_foot_ori = left_foot_ori;
          
            break;}
          
            case Right:{
            // get stance and swing foot pos
            swing_foot_pos << left_foot_toe_pos;
            stance_foot_pos << right_foot_pos;

            swing_foot_ori = left_foot_ori;
            stance_foot_ori = right_foot_ori;
           
            break;}
        }
        // std::cout << "com pos: " << y_act.segment(0,3) << std::endl;
        // std::cout << "stance foot pos: " << stance_foot_pos << std::endl;

        y_act.segment(0,3) = y_act.segment(0,3) - stance_foot_pos;
        y_act.segment(3,3) = pelv_act;
        y_act.segment(6,3) = swing_foot_pos-stance_foot_pos;
        y_act.segment(9,3) = swing_foot_ori;
        y_act.segment(12,3) = stance_foot_pos;
        y_act.segment(15,3) = stance_foot_ori;

        // std::cout << "q pos: " << std::endl;

        // for(int i=0; i<model->nq; i++){
        //     std::cout << data->qpos[i] << " ";
        // }
        // // std::cout << std::endl;
        // // std::cout << "y_act: " << y_act.transpose() << std::endl;
        // std::terminate();

        }


    void getStairJacobian(matrix_t& J_y_h, const mjModel* model, mjData* data,int whichStance){
        //call mj_jacSite to get the jacobian
        // void mj_jacSite(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int site);
        double jacp[3*18];
        double jacr[3*18];
        // mj_kinematics(model, data);
        // mj_comPos(model, data);

        int torso_id = mj_name2id(model, mjOBJ_BODY, "torso");
        mj_jacSubtreeCom(model, data, jacp, torso_id);

        //extract the jacobian
        matrix_t J = matrix_t::Zero(3,18);
        for(int i=0; i<3; i++){
            for(int j=0; j<18; j++){
                J(i,j) = jacp[i*18+j];
            }
        }

        mj_jacSite(model, data,  jacp,jacr, mj_name2id(model, mjOBJ_SITE, "left_sole_frame"));
        matrix_t J_left = matrix_t::Zero(6,18);
        for(int i=0; i<3; i++){
            for(int j=0; j<18; j++){
                J_left(i,j) = jacp[i*18+j];
            }
        }
        for(int i=0; i<3; i++){
            for(int j=0; j<18; j++){
                J_left(i+3,j) = jacr[i*18+j];
            }
        }

        mj_jacSite(model, data,  jacp,jacr, mj_name2id(model, mjOBJ_SITE, "left_toe_frame"));
        matrix_t J_left_toe = matrix_t::Zero(6,18);
        for(int i=0; i<3; i++){
            for(int j=0; j<18; j++){
                J_left_toe(i,j) = jacp[i*18+j];
            }
        }
        for(int i=0; i<3; i++){
            for(int j=0; j<18; j++){
                J_left_toe(i+3,j) = jacr[i*18+j];
            }
        }


        mj_jacSite(model, data,  jacp,jacr, mj_name2id(model, mjOBJ_SITE, "right_sole_frame"));
        matrix_t J_right = matrix_t::Zero(6,18);
        for(int i=0; i<3; i++){
            for(int j=0; j<18; j++){
                J_right(i,j) = jacp[i*18+j];
            }
        }
        for(int i=0; i<3; i++){
            for(int j=0; j<18; j++){
                J_right(i+3,j) = jacr[i*18+j];
            }
        }

        mj_jacSite(model, data,  jacp,jacr, mj_name2id(model, mjOBJ_SITE, "right_toe_frame"));
        matrix_t J_right_toe = matrix_t::Zero(6,18);
        for(int i=0; i<3; i++){
            for(int j=0; j<18; j++){
                J_right_toe(i,j) = jacp[i*18+j];
            }
        }
        for(int i=0; i<3; i++){
            for(int j=0; j<18; j++){
                J_right_toe(i+3,j) = jacr[i*18+j];
            }
        }

        mj_jacBody(model, data, jacp, jacr, mj_name2id(model, mjOBJ_BODY, "torso"));
        matrix_t J_torso = matrix_t::Zero(3,18);
        for(int i=0; i<3; i++){
            for(int j=0; j<18; j++){
                J_torso(i,j) = jacr[i*18+j];
            }
        }
        // std::cout << "J_COM: " << J << std::endl;
        // std::cout << "J_left: " << J_left << std::endl;
        // std::cout << "J_right: " << J_right << std::endl;
        // std::terminate();
        switch(whichStance){
            case Left:{
                J_y_h.block(0,0,3,18) = J - J_left.block(0,0,3,18); //com2left
                J_y_h.block(3,0,3,18) = J_torso; //pelv
                J_y_h.block(6,0,3,18) = J_right_toe.block(0,0,3,18) - J_left.block(0,0,3,18); //right2left
                J_y_h.block(9,0,3,18) = J_right.block(3,0,3,18); //right_foot_ori

                J_y_h.block(12,0,6,18) = J_left;
                break;
            }
            case Right:{
                J_y_h.block(0,0,3,18) = J - J_right.block(0,0,3,18); //com2right
                J_y_h.block(3,0,3,18) = J_torso; //pelv
                J_y_h.block(6,0,3,18) = J_left_toe.block(0,0,3,18) - J_right.block(0,0,3,18); //left2right
                J_y_h.block(9,0,3,18) = J_left.block(3,0,3,18); //left_foot_ori

                J_y_h.block(12,0,6,18) = J_right;
                break;
            }
        }
    }


   void getY(vector_t& y_act,const mjModel* model, mjData* data,int whichStance) {
        // Get the actual CoM position
        // double* com = SensorByName(model, data, "torso_subcom");
        // y_act.segment(0,3) << com[0], com[1], com[2];
        // mj_comPos(model, data);

        double com[3] = {0.0};
        mju_addTo(com, data->subtree_com, 3);
        y_act.segment(0,3) << com[0], com[1], com[2];

        // std::cout << "com pos: " << com[0] << " " << com[1] << " " << com[2] << std::endl;

        //Get Pelvis Orientation
        vector_t quat_act = vector_t::Zero(4);
        quat_act << data->qpos[3],data->qpos[4],data->qpos[5],data->qpos[6];
        vector_t pelv_act = utils::quat2eulXYZ(quat_act);
        
        //Get Pelvis Angular Velocity
        int left_foot_id = mj_name2id(model, mjOBJ_SITE, "left_heel_frame");
        int right_foot_id = mj_name2id(model, mjOBJ_SITE, "right_heel_frame");

        //extract site pos and ori from data.site_xpos and data.site_xmat
        vector_t left_foot_pos = vector_t::Zero(3);
        vector_t right_foot_pos = vector_t::Zero(3);
        vector_t left_foot_ori = vector_t::Zero(3);
        vector_t right_foot_ori = vector_t::Zero(3);

        for(int i=0; i<3; i++){
            left_foot_pos[i] = data->site_xpos[left_foot_id*3+i];
            right_foot_pos[i] = data->site_xpos[right_foot_id*3+i];
        }

        //convert to rotation matrix to euler
        matrix_t left_foot_rot = matrix_t::Zero(3,3);
        matrix_t right_foot_rot = matrix_t::Zero(3,3);
        for(int i=0; i<3; i++){
            for(int j=0; j<3; j++){
                left_foot_rot(i,j) = data->site_xmat[left_foot_id*9+i*3+j];
                right_foot_rot(i,j) = data->site_xmat[right_foot_id*9+i*3+j];
            }
        }

        left_foot_ori = utils::quat2eulXYZ(utils::quatFrom3x3(left_foot_rot));
        right_foot_ori = utils::quat2eulXYZ(utils::quatFrom3x3(right_foot_rot));

        vector_t stance_foot_pos = vector_t::Zero(3);
        vector_t swing_foot_pos = vector_t::Zero(3);
        vector_t swing_foot_ori = vector_t::Zero(3);
        vector_t stance_foot_ori = vector_t::Zero(3);

        switch(whichStance){
            case Left:{
            // get stance and swing foot pos
            swing_foot_pos << right_foot_pos;
            stance_foot_pos << left_foot_pos;

            swing_foot_ori = right_foot_ori;
            stance_foot_ori = left_foot_ori;
          
            break;}
          
            case Right:{
            // get stance and swing foot pos
            swing_foot_pos << left_foot_pos;
            stance_foot_pos << right_foot_pos;

            swing_foot_ori = left_foot_ori;
            stance_foot_ori = right_foot_ori;
           
            break;}
        }
        // std::cout << "com pos: " << y_act.segment(0,3) << std::endl;
        // std::cout << "stance foot pos: " << stance_foot_pos << std::endl;

        y_act.segment(0,3) = y_act.segment(0,3) - stance_foot_pos;
        y_act.segment(3,3) = pelv_act;
        y_act.segment(6,3) = swing_foot_pos-stance_foot_pos;
        y_act.segment(9,3) = swing_foot_ori;
        y_act.segment(12,3) = stance_foot_pos;
        y_act.segment(15,3) = stance_foot_ori;

        // std::cout << "q pos: " << std::endl;

        // for(int i=0; i<model->nq; i++){
        //     std::cout << data->qpos[i] << " ";
        // }
        // // std::cout << std::endl;
        // // std::cout << "y_act: " << y_act.transpose() << std::endl;
        // std::terminate();

        }


    void getJacobian(matrix_t& J_y_h, const mjModel* model, mjData* data,int whichStance){
        //call mj_jacSite to get the jacobian
        // void mj_jacSite(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int site);
        double jacp[3*18];
        double jacr[3*18];
        // mj_kinematics(model, data);
        // mj_comPos(model, data);

        int torso_id = mj_name2id(model, mjOBJ_BODY, "torso");
        mj_jacSubtreeCom(model, data, jacp, torso_id);

        //extract the jacobian
        matrix_t J = matrix_t::Zero(3,18);
        for(int i=0; i<3; i++){
            for(int j=0; j<18; j++){
                J(i,j) = jacp[i*18+j];
            }
        }

        mj_jacSite(model, data,  jacp,jacr, mj_name2id(model, mjOBJ_SITE, "left_heel_frame"));
        matrix_t J_left = matrix_t::Zero(6,18);
        for(int i=0; i<3; i++){
            for(int j=0; j<18; j++){
                J_left(i,j) = jacp[i*18+j];
            }
        }
        for(int i=0; i<3; i++){
            for(int j=0; j<18; j++){
                J_left(i+3,j) = jacr[i*18+j];
            }
        }

        mj_jacSite(model, data,  jacp,jacr, mj_name2id(model, mjOBJ_SITE, "right_heel_frame"));
        matrix_t J_right = matrix_t::Zero(6,18);
        for(int i=0; i<3; i++){
            for(int j=0; j<18; j++){
                J_right(i,j) = jacp[i*18+j];
            }
        }
        for(int i=0; i<3; i++){
            for(int j=0; j<18; j++){
                J_right(i+3,j) = jacr[i*18+j];
            }
        }

        mj_jacBody(model, data, jacp, jacr, mj_name2id(model, mjOBJ_BODY, "torso"));
        matrix_t J_torso = matrix_t::Zero(3,18);
        for(int i=0; i<3; i++){
            for(int j=0; j<18; j++){
                J_torso(i,j) = jacr[i*18+j];
            }
        }
        // std::cout << "J_COM: " << J << std::endl;
        // std::cout << "J_left: " << J_left << std::endl;
        // std::cout << "J_right: " << J_right << std::endl;
        // std::terminate();
        switch(whichStance){
            case Left:{
                J_y_h.block(0,0,3,18) = J - J_left.block(0,0,3,18); //com2left
                J_y_h.block(3,0,3,18) = J_torso; //pelv
                J_y_h.block(6,0,3,18) = J_right.block(0,0,3,18) - J_left.block(0,0,3,18); //right2left
                J_y_h.block(9,0,3,18) = J_right.block(3,0,3,18); //right_foot_ori

                J_y_h.block(12,0,6,18) = J_left;
                break;
            }
            case Right:{
                J_y_h.block(0,0,3,18) = J - J_right.block(0,0,3,18); //com2right
                J_y_h.block(3,0,3,18) = J_torso; //pelv
                J_y_h.block(6,0,3,18) = J_left.block(0,0,3,18) - J_right.block(0,0,3,18); //left2right
                J_y_h.block(9,0,3,18) = J_left.block(3,0,3,18); //left_foot_ori

                J_y_h.block(12,0,6,18) = J_right;
                break;
            }
        }
    }

}//namespace taskspace_utils


namespace utils {


    vector_t quatFrom3x3(const Eigen::Matrix3d& m) {
        double w = std::sqrt(1.0 + m(0, 0) + m(1, 1) + m(2, 2)) / 2.0;
        double x = (m(2, 1) - m(1, 2)) / (w * 4);
        double y = (m(0, 2) - m(2, 0)) / (w * 4);
        double z = (m(1, 0) - m(0, 1)) / (w * 4);
        vector_t v(4);
        v << w, x, y, z;
        return v;
    }



    matrix_t R_z(double z)
{
        matrix_t R = matrix_t::Zero(3,3);
        
        R(0, 0) = cos(z);
        R(0, 1) = -sin(z);
        R(1, 0) = sin(z);
        R(1, 1) = cos(z);
        R(2, 2) = 1.0;

    return R;
}

    void eulerXYZ(Eigen::Quaterniond &q, Eigen::EulerAnglesXYZd &euler){
    // double q0, q1, q2, q3;

    // q0 = q.w();
    // q1 = q.x();
    // q2 = q.y();
    // q3 = q.z();

    // euler.alpha() = atan2(2.0*q0*q1 - 2.0*q2*q3, q0*q0 - q1*q1 - q2*q2 + q3*q3);
    // euler.beta()  = asin(2.0 * ( q0*q2 + q3*q1 ));
    // euler.gamma() = atan2(2.0 * ( q0*q3 - q1*q2 ), q0*q0 + q1*q1 - q2*q2 - q3*q3);

    // from matlab function quat2eul.m
    double qx,qy,qz,qw;
    qw = q.w();
    qx = q.x();
    qy = q.y();
    qz = q.z();

    double aSinInput = 2*(qx*qz + qy*qw);
    if (aSinInput > 1)
        aSinInput = 1;
    if ((aSinInput < -1))   
        aSinInput = -1;

        
    euler.alpha() =  atan2( -2*(qy*qz - qx*qw), pow(qw,2) - pow(qx,2) - pow(qy,2) + pow(qz,2) );
    euler.beta()  = asin( aSinInput );
    euler.gamma() = atan2( -2*(qx*qy - qz*qw), pow(qw,2) + pow(qx,2) - pow(qy,2) - pow(qz,2) );

    }

    void eulerXYZ(Eigen::Matrix3d &R, Eigen::EulerAnglesXYZd &euler){
        double y1, x1, z1, x, y, z;
        if ( abs(abs(R(2,0)) - 1.0) > 0.00000001 ) {
            y1 = -asin(R(2,0));
            //y2 = 3.14159265359 - y1;

            x1 = atan2(R(2,1)/cos(y1), R(2,2)/cos(y1));
            //x2 = atan2(R(2,1)/cos(y2), R(2,2)/cos(y2));

            z1 = atan2(R(1,0)/cos(y1), R(0,0)/cos(y1));
            //z2 = atan2(R(1,0)/cos(y2), R(0,0)/cos(y2));

            euler.alpha() = x1;
            euler.beta()  = y1;
            euler.gamma() = z1;
        } else {
            z = 0.0;
            if ( abs(R(2,0) + 1.0) < 0.00000001 ) {
                y = 3.14159265359/2.0;
                x = z + atan2(R(0,1), R(0,2));
            } else {
                y = -3.14159265359/2.0;
                x = -z + atan2(-R(0,1), -R(0,2));
            }
            euler.alpha() = x;
            euler.beta()  = y;
            euler.gamma() = z;
        }
    }

    Eigen::Matrix3d skew(Eigen::Vector3d &v) {
        Eigen::Matrix3d sk;

        sk << 0.0, -v(2), v(1),
            v(2), 0.0, -v(0),
            -v(1), v(0), 0.0;
        return sk;
    }


    quat_t eulerXYZ2quat(vector_t eul){

        scalar_t c[3] = {cos(eul[0]/2),cos(eul[1]/2),cos(eul[2]/2)};
        scalar_t s[3] = {sin(eul[0]/2),sin(eul[1]/2),sin(eul[2]/2)};
        vector_t q;
        q.resize(4);//wxyz 
        q << c[0]*c[1]*c[2] - s[0]*s[1]*s[2],
            s[0]*c[1]*c[2] +c[0]*s[1]*s[2],
            -s[0]*c[1]*s[2]+c[0]*s[1]*c[2],
            c[0]*c[1]*s[2]+s[0]*s[1]*c[2];
        quat_t quat = quat_t(q[0],q[1],q[2],q[3]);
    return quat;
    }


    void hatmap(const Eigen::Vector3d &w, Eigen::Matrix3d &skew){
        skew.setZero();
        skew(1,0) = w(2);
        skew(0,1) = -w(2);
        skew(2,0) = -w(1);
        skew(0,2) = w(1);
        skew(1,2) = -w(0);
        skew(2,1) = w(0);
    }

    vector_t eulRates2omegaXYZ(vector_t eul,vector_t eul_rates){
        scalar_t y = eul[1];
        scalar_t z = eul[2];
        matrix_3t M;
        M << cos(y)*cos(z), sin(z), 0,
            -cos(y)*sin(z), cos(z), 0,
            sin(y), 0, 1;
        vector_t omega = M * eul_rates;
        return omega;
    }

    vector_t omega2eulRates(vector_t eul,vector_t omega){
        scalar_t y = eul[1];
        scalar_t z = eul[2];
        matrix_3t M;
        M  << cos(z)/cos(y), -sin(z)/cos(y), 0,
            sin(z), cos(z), 0,
            -cos(z)*tan(y), sin(z)*tan(y),1;
        vector_t eul_rates = M * omega;
        return eul_rates;

    }


    vector_t eulRates2omegaZYX(vector_t eul,vector_t eul_rates){
        scalar_t x = eul[0];
        scalar_t y = eul[1];
        matrix_3t M;
        M << 1, 0, -sin(y),
            0, cos(x), sin(x)*cos(y),
            0, -sin(x), cos(x)*cos(y);
        vector_t omega = M * eul_rates;
        return omega;
    }

    

    vector_t quat2eulXYZ(vector_t quat){
        //should assume wxyz convention
        //z1_2 should be qy,1 should be x, w should be 0
        scalar_t b;
        scalar_t z1_idx_0;
        scalar_t z1_idx_1;
        scalar_t z1_idx_2;
        scalar_t aSinInput;
        scalar_t eul_tmp;
        scalar_t b_eul_tmp;
        scalar_t c_eul_tmp;
        scalar_t d_eul_tmp;

        vector_t eul = vector_t::Zero(3);
        b = 1.0 / std::sqrt(((quat[0] * quat[0] + quat[1] * quat[1]) + quat[2] * quat
                            [2]) + quat[3] * quat[3]);
        z1_idx_0 = quat[0] * b;
        z1_idx_1 = quat[1] * b;
        z1_idx_2 = quat[2] * b;
        b *= quat[3];
        aSinInput = 2.0 * (z1_idx_1 * b + z1_idx_2 * z1_idx_0);
        if (aSinInput > 1.0) {
            aSinInput = 1.0;
        }

        if (aSinInput < -1.0) {
            aSinInput = -1.0;
        }

        eul_tmp = z1_idx_0 * z1_idx_0;
        b_eul_tmp = z1_idx_1 * z1_idx_1;
        c_eul_tmp = z1_idx_2 * z1_idx_2;
        d_eul_tmp = b * b;
        eul[0] = atan2(-2.0 * (z1_idx_2 * b - z1_idx_1 * z1_idx_0), ((eul_tmp
            - b_eul_tmp) - c_eul_tmp) + d_eul_tmp);
        eul[1] = asin(aSinInput);
        eul[2] = atan2(-2.0 * (z1_idx_1 * z1_idx_2 - b * z1_idx_0), ((eul_tmp
            + b_eul_tmp) - c_eul_tmp) - d_eul_tmp);
        return eul;
    }

    
    vector_t quat2eulZYX(vector_t quat){ //return in rpy; intrinsic
        scalar_t b;
        scalar_t z1_idx_0;
        scalar_t z1_idx_1;
        scalar_t z1_idx_2;
        scalar_t aSinInput;
        scalar_t eul_tmp;
        scalar_t b_eul_tmp;
        scalar_t c_eul_tmp;
        scalar_t d_eul_tmp;

        vector_t eul = vector_t::Zero(3);
        b = 1.0 / std::sqrt(((quat[0] * quat[0] + quat[1] * quat[1]) + quat[2] * quat
                            [2]) + quat[3] * quat[3]);
        z1_idx_0 = quat[0] * b;
        z1_idx_1 = quat[1] * b;
        z1_idx_2 = quat[2] * b;
        b *= quat[3];
        aSinInput = 2.0 * (z1_idx_1 * b + z1_idx_2 * z1_idx_0);
        if (aSinInput > 1.0) {
            aSinInput = 1.0;
        }

        if (aSinInput < -1.0) {
            aSinInput = -1.0;
        }

        eul_tmp = z1_idx_0 * z1_idx_0;
        b_eul_tmp = z1_idx_1 * z1_idx_1;
        c_eul_tmp = z1_idx_2 * z1_idx_2;
        d_eul_tmp = b * b;
        eul[2] = atan2(-2.0 * (z1_idx_2 * b - z1_idx_1 * z1_idx_0), ((eul_tmp
            - b_eul_tmp) - c_eul_tmp) + d_eul_tmp);
        eul[1] = std::asin(aSinInput);
        eul[0] = atan2(-2.0 * (z1_idx_1 * z1_idx_2 - b * z1_idx_0), ((eul_tmp
            + b_eul_tmp) - c_eul_tmp) - d_eul_tmp);
        return eul;
    }


    void R_XYZ(vector_t euler_angle, Eigen::Matrix<double,3,3> &R){

        double roll = euler_angle(0);
        double pitch = euler_angle(1);
        double yaw = euler_angle(2);

        double sroll = sin(roll);
        double croll = cos(roll);
        double spitch = sin(pitch);
        double cpitch = cos(pitch);
        double syaw = sin(yaw);
        double cyaw = cos(yaw);

        //Rx x Ry x Rz: intrinsic xyz or extrinsic ZYX
        R(0,0) = cpitch*cyaw;
        R(1,0) = croll*syaw + cyaw*spitch*sroll;
        R(2,0) = sroll*syaw - croll*cyaw*spitch;
        R(0,1) = -cpitch*syaw;
        R(1,1) = croll*cyaw - spitch*sroll*syaw;
        R(2,1) = cyaw*sroll + croll*spitch*syaw;
        R(0,2) = spitch;
        R(1,2) = -cpitch*sroll;
        R(2,2) = cpitch*croll;
}

    vector_t Rot2eulXYZ(matrix_t R){
            //exist issue with this implementation
        vector_t eul = vector_t::Zero(3);
        double sy = sqrt(R(2,2)*R(2,2) + R(1,2)*R(1,2));    
        eul << atan2(R(1,2), R(2,2)), atan2(-R(0,2), sy), atan2(R(0,1), R(0,0));
        return eul;
    }


    

    vector_t QuatXYZWToEulerZYX(vector_t quat_xyzw)
    {
        vector_t quat_wxyz = vector_t::Zero(4);
        quat_wxyz(0) = quat_xyzw(3);
        quat_wxyz(1) = quat_xyzw(0);
        quat_wxyz(2) = quat_xyzw(1);
        quat_wxyz(3) = quat_xyzw(2);

        vector_t euler_zyx = QuatWXYZToEulerZYX(quat_wxyz);

        return euler_zyx;
    }

    vector_t QuatWXYZToEulerZYX(vector_t quat_wxyz)
    {
        double q_w = quat_wxyz(0);
        double q_x = quat_wxyz(1);
        double q_y = quat_wxyz(2);
        double q_z = quat_wxyz(3);

        vector_t euler_zyx = vector_t::Zero(3);
        // Assuming the quaterinion is normalized
        // roll
        euler_zyx(0) = std::atan2(2*(q_w*q_x + q_y*q_z), 1 - 2*(q_x*q_x + q_y*q_y)); 
        // pitch
        euler_zyx(1) = -M_PI/2 + 2*std::atan2(std::sqrt(1 + 2*(q_w*q_y - q_x*q_z)), std::sqrt(1 - 2*(q_w*q_y - q_x*q_z)));
        // yaw
        euler_zyx(2) = std::atan2(2*(q_w*q_z + q_x*q_y), 1 - 2*(q_y*q_y + q_z*q_z));

        return euler_zyx;
    }

    vector_t EulerZYXToQuatWXYZ(vector_t euler_zyx)
    {
        double roll = euler_zyx(0);
        double pitch = euler_zyx(1);
        double yaw = euler_zyx(2);

        double cr = std::cos(roll/2);
        double sr = std::sin(roll/2);
        double cp = std::cos(pitch/2);
        double sp = std::sin(pitch/2);
        double cy = std::cos(yaw/2);
        double sy = std::sin(yaw/2);

        double q_w, q_x, q_y, q_z;
        q_w = cr * cp * cy + sr * sp * sy;
        q_x = sr * cp * cy - cr * sp * sy;
        q_y = cr * sp * cy + sr * cp * sy;
        q_z = cr * cp * sy - sr * sp * cy;
        
        vector_t quat_wxyz = vector_t::Zero(4);

        quat_wxyz(0) = q_w;
        quat_wxyz(1) = q_x;
        quat_wxyz(2) = q_y;
        quat_wxyz(3) = q_z;

        return quat_wxyz;
    }

    vector_t EulerZYXToQuatXYZW(vector_t euler_zyx)
{
        vector_t quat_wxyz = EulerZYXToQuatWXYZ(euler_zyx);

        vector_t quat_xyzw = vector_t::Zero(4);
        quat_xyzw(0) = quat_wxyz(1);
        quat_xyzw(1) = quat_wxyz(2);
        quat_xyzw(2) = quat_wxyz(3);
        quat_xyzw(3) = quat_wxyz(0);

    return quat_xyzw;
}


    // Convert Euler rates to angular velocity
    vector_t eulerRatesToAngularVelocity(const Eigen::Vector3d& eulerRates, const Eigen::Vector3d& euler) {
        // Create the rotation matrix from Euler angles
        Eigen::Matrix3d rotationMatrix;
        rotationMatrix = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());

        // Compute the angular velocity using the rotation matrix and Euler rates
        Eigen::Vector3d angularVelocity = rotationMatrix * eulerRates;

        return angularVelocity;
    }


}




namespace coeffUtils{

    // reshape bezier coefficients into matrix
    matrix_t reshapeCoeff(int deg, std::vector<scalar_t> coeff_b_vec, std::vector<scalar_t> coeff_jt_vec){
        matrix_t coeff_mat = matrix_t::Zero(18,deg+1);

        // Go through base coordinates
        coeff_mat.block(0,0,6,deg+1) = matrixUtils::reshapeMatrix(6, deg+1, coeff_b_vec);

        // Go through joint coordinates
        coeff_mat.block(6,0,12,deg+1) =  matrixUtils::reshapeMatrix(12, deg+1, coeff_jt_vec);
      
        return coeff_mat;
    }

    matrix_t remapSymmetric(matrix_t coeff_mat){
        // Initialize remapped as zeros
        matrix_t coeff_remap = matrix_t::Zero(18,8);

        // Populate remapped coefficients by flipping left and right legs
        coeff_remap.block(0, 0, 6, 8) = coeff_mat.block(0, 0, 6, 8);
        coeff_remap.block(6, 0, 6, 8) = coeff_mat.block(12, 0, 6, 8);
        coeff_remap.block(12, 0, 6, 8) = coeff_mat.block(6, 0, 6, 8);

        // Negate certain states (ffy, ffroll, ffyaw, left_frontalhip, left_transversehip, left_henkeankle, right_frontalhip, right_transversehip, right_henkeankle)
        scalar_t jts2negate[9] = {1, 3, 5, 6, 7, 11, 12, 13, 17};
        for (int j = 0; j < 9; j++)
        {
            int idx = jts2negate[j];
            coeff_remap.block(idx, 0, 1, 8) = -1 * coeff_remap.block(idx, 0, 1, 8);
        }
        return coeff_remap;
    }

}


namespace matrixUtils{
    matrix_t reshapeMatrix(int numRow, int numCol, std::vector<scalar_t> vec){
        matrix_t matrix = matrix_t::Zero(numRow,numCol);
         for (int i = 0; i < numCol; i++)
        {
            for (int j = 0; j < numRow; j++)
            {
                matrix(j, i) = vec[i * numRow + j];
            }
        }
        return matrix;
    }

    matrix_t reshapeMatrix(int numRow, int numCol, const double* vec){
        matrix_t matrix = matrix_t::Zero(numRow,numCol);
         for (int i = 0; i < numCol; i++)
        {
            for (int j = 0; j < numRow; j++)
            {
                matrix(j, i) = vec[i * numRow + j];
            }
        }
        return matrix;
    }

    vector_t mapVec2Eigen(std::vector<scalar_t> vec){
        vector_t eigen_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec.data(), vec.size());

        return eigen_vec;
    }

    std::vector<double> flattenMatrix(const matrix_t& matrix) {
        std::vector<double> vec;
        int numRow = matrix.rows();
        int numCol = matrix.cols();

        // Reserve space to improve performance
        vec.reserve(numRow * numCol);

        // Iterate over the matrix elements in column-major order
        for (int i = 0; i < numCol; i++) {
            for (int j = 0; j < numRow; j++) {
                vec.push_back(matrix(j, i));
            }
        }

        return vec;
    }

}


namespace jointUtils{
    vector_t remapJoint(vector_t &q){
        scalar_t jts2negate[9] = {1, 3, 5, 6, 7, 11, 12, 13, 17};
        vector_t q_remap = q;
        q_remap.segment(6,6) = q.segment(12,6);
        q_remap.segment(12,6) = q.segment(6,6);
        for (int j=0;j<9;j++){ 
            int idx = jts2negate[j];
            q_remap(idx) = -1 * q_remap(idx);
        }
        return q_remap;
    }

     vector_t remapActuatedJoint(vector_t &q){
        scalar_t jts2negate[6] = {0, 1, 5, 6, 7, 11};
        vector_t q_remap = q;
        q_remap.segment(0,6) = q.segment(6,6);
        q_remap.segment(6,6) = q.segment(0,6);
        for (int j=0;j<6;j++){ 
            int idx = jts2negate[j];
            q_remap(idx) = -1 * q_remap(idx);
        }
        return q_remap;
    }

    vector_t convertFrostToPino(vector_t &q_frost){
        vector_t q_pino = vector_t::Zero(19);
        q_pino.segment(7,12) = q_frost.segment(6,12);
        q_pino.segment(0,3) = q_frost.segment(0,3);
        q_pino.segment(3,4) = utils::eulerXYZ2quat(q_frost.segment(3,3)).coeffs();
        return q_pino;
    }


}


