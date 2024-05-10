#ifndef EXO_CONSTANTS_HPP
#define EXO_CONSTANTS_HPP

typedef enum {

    nMotors = 12,
    nYstanding = 6, // standing output
    nYwalking = 12,
    nState = 18,

    // states 
    BasePosX = 0,
    BasePosY = 1,
    BasePosZ = 2,
    BaseRotX = 3,
    BaseRotY = 4,
    BaseRotZ = 5,
    LeftFrontalHip =6,
    LeftTransverseHip =7,
    LeftSagittalHip =8,
    LeftSagittalKnee =9,
    LeftSagittalAnkle =10,
    LeftHenkeAnkle =11,
    RightFrontalHip =12,
    RightTransverseHip=13,
    RightSagittalHip=14,
    RightSagittalKnee=15,
    RightSagittalAnkle=16,
    RightHenkeAnkle=17,


    //pinocchio frame id
    L_front_hip_id = 3,
    L_trans_hip_id = 5,
    L_sag_hip_id = 7,
    L_sag_knee_id = 9,
    L_sag_ank_id = 11,
    L_henke_ank_id = 13,

    R_front_hip_id = 31,
    R_trans_hip_id = 33,
    R_sag_hip_id = 35,
    R_sag_knee_id = 37,
    R_sag_ank_id = 39,
    R_henke_ank_id = 41,


    // Walking outputs
    y_comx_idx = 0,
    y_comy_idx = 1,
    y_comz_idx = 2,
    y_ori_r_idx = 3,
    y_ori_p_idx = 4,
    y_ori_y_idx = 5,
    y_swposx_idx = 6,
    y_swposy_idx = 7,
    y_swposz_idx = 8,
    y_sw_r_idx = 9,
    y_sw_p_idx = 10,
    y_sw_y_idx = 11,

    // StanceBasicEnum
    InTheAir =-1,
    Left =0,
    Right =1,
    DS =2,
    LeftDS=3,
    RightDS =4,

    //StateBasicEnum
    ToLoading = 0,
    Loading = 1,
    ToStopped = 2,
    Stopped = 3,
    WantToStart = 4,
    StartingStep = 5,
    Walking = 6,
    WantToStop = 7,
    StoppingStep = 8,

    //Button Array
    PHYSIO_STAND_TO_SIT = 0,
    PHYSIO_SITTING = 1,
    PHYSIO_TRANSFER = 2,
    REMOTE_STANDSTILL = 3,
    REMOTE_WALK = 4,
    REMOTE_TURN_AROUND = 5,
    REMOTE_REPOSITIONING = 6,
    LEFTLEG_TRANSFER = 7,
    RIGHTLEG_TRANSFER = 8,

    //com_xy_replan
    COMX_REPLAN = 0,
    COMY_REPLAN = 1,
    COMXY_REPLAN = 2,

}ExoConstant;

#endif // Exo_CONSTANTS_HPP