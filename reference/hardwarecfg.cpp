/*
 * @Author: Rose
 * @Date: 2024-04-27 21:00:14
 * @LastEditors: linzhuyue 
 * @LastEditTime: 2024-12-23 19:03:10
 * @FilePath: /cyan_lowlevel_sdk/src/hardware/hardwarecfg.cpp
 * @Description: Writed In SH.
 * 
 * Copyright (c) 2024 by Rose, All Rights Reserved. 
 */
#include "color_control.hpp"
#include <ostream>
#include <vector>
#include <iostream>
#include <cmath>
#include "hardwarecfg.hpp"
#include "rt_rc_interface.h"
#include "filter.hpp"
#include "MahonyAHRS.h"

extern rc_control_settings rc_control;
namespace hardware_cfg{
    HardwareCFG::HardwareCFG():
        Controller2Robot("udpm://239.255.76.67:7667?ttl=255"),
        Robot2Controller("udpm://239.255.76.67:7667?ttl=255"),
        // receiveTask(&taskManager, 0.002, "recv_task", &HardwareCFG::read, this),
        // sendTask   (&taskManager, 0.002, "send_task", &HardwareCFG::write, this),
        lcmreceiveTask(&taskManager, 0.001, "lcmrecv_task", &HardwareCFG::_thread_lcmrec_run, this),
        lcmsendTask   (&taskManager, 0.001, "lcmsend_task", &HardwareCFG::_thread_lcmsen_run, this),
        imuTask    (&taskManager, 0.001, "imu_task", &HardwareCFG::_thread_imu_run, this),
        // gampadTask (&taskManager, 0.02, "gamepad_task", &HardwareCFG::thread_run_xbox, this)
        sdkStateTask(&taskManager, 0.001, "sdk_state_task", &HardwareCFG::_thread_sdk_state_run, this)
        {
            sdkStateTask.start();
        }
bool HardwareCFG::get_eterInitState(){
    return init_state;
}
void HardwareCFG::setPeoridFreq(const int freq){
    peroid_freq_ = freq;
}
void HardwareCFG::start_sub_thread(){

    Controller2Robot.subscribe( "controller2robot_legs", &HardwareCFG::handleControllerlegs2RobotLCM, this);
    Controller2Robot.subscribe( "controller2robot_ahw",  &HardwareCFG::handleControllerAHW2RobotLCM, this);
    Controller2Robot.subscribe( "controller2robot_ahw_leg", &HardwareCFG::handleControllerAHWLEG2RobotLCM, this);
    Controller2Robot.subscribe( "controller2gamepad", &HardwareCFG::handleController2GamepadLCM, this);
        
    // receiveTask.start();
    // sendTask.start();
    lcmreceiveTask.start();
    lcmsendTask.start();
    imuTask.start();
    // gampadTask.start();
    std::cout<<output_color::green<<"LCM Initialized!!"<<output_color::normal<<std::endl;
}
bool HardwareCFG::init()
{
    //step1: Initlized Ethercat
    // int ec_slavecount = EtherCAT_Init("enx68da73acd09a");
    // std::cout << "Start Initilized Ethercat!!" << std::endl;
    // if (ec_slavecount <= 0)
    // {
    //     std::cout << "No slave found, will be quited!!!" << std::endl;
    //     init_state = false;
    //     return false;
    // }
    // step2: init IMU
        try
        {
            com_port = std::atoi("/dev/ttyUSB0");
            baudrate = 921600;
            imu_data = imu_data_struct();
            if(lordimu_.tryInit(com_port, baudrate)){
                std::cout<<"Lord IMU initialized OK!"<<std::endl;
                
            }else{
                imu_open_ok=false;
                std::cout<<output_color::red<<"IMU initialized failure!"<<output_color::normal<<std::endl;
                return false;
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

        start_sub_thread();
    // EtherCAT_Send_Command((YKSMotorData*)yksSendcmdzero_);
    // usleep(1000);
    // EtherCAT_Get_State();
    //step3: init xbox
    // try
    // {
    //     port_xbox=init_xbox_js();
    //     std::cout<<"xbox port initialized ok!"<<std::endl;
    // }
    // catch(const std::exception& e)
    // {
    //     std::cerr << e.what()<<" port xbox initialized failed" << '\n';
    // }
    setZero();
    init_state=true;
    return true;
    
}
void HardwareCFG::setZero(){
    // 0-18 12 arms,3 waist 3 head
    for (int i = 0; i < ArmWaistHeadNum; i++)
    {
        cyan_armwaisthead_cmd_.tau_ff  [i]    =  0.0f; 
        cyan_armwaisthead_cmd_.q_des   [i]    =  0.0f;
        cyan_armwaisthead_cmd_.qd_des  [i]    =  0.0f;
        cyan_armwaisthead_cmd_.kp_joint[i]    =  0.0f;
        cyan_armwaisthead_cmd_.kd_joint[i]    =  0.0f;
        cyan_armwaisthead_cmd_.toruqe_range_ratio[i] = 0.0f;
        cyan_armwaisthead_data_.q      [i]    =  0.0f;  
        cyan_armwaisthead_data_.qd     [i]    =  0.0f;
        cyan_armwaisthead_data_.tauIq  [i]    =  0.0f;
        cyan_armwaisthead_data_.tauCom  [i]    =  0.0f;
        if(i<12)cyan_armwaisthead_data_.ext_contact[i] = 0.0f;

        cyan_armwaisthead_mid_data_.q      [i]    =  0.0f;  
        cyan_armwaisthead_mid_data_.qd     [i]    =  0.0f;
        cyan_armwaisthead_mid_data_.tauIq  [i]    =  0.0f;
        cyan_armwaisthead_mid_data_.tauCom  [i]    =  0.0f;
        if(i<12)cyan_armwaisthead_mid_data_.ext_contact[i] = 0.0f;
    }
    // leg 0-12
    for (int i = 0; i < LegJointNUM; i++)
    {
        cyan_hand_cmd_.tau_ff  [i] = 0.0f;  
        cyan_hand_cmd_.q_des   [i] = 0.0f; 
        cyan_hand_cmd_.qd_des  [i] = 0.0f;
        cyan_hand_cmd_.kp_joint[i] = 0.0f;
        cyan_hand_cmd_.kd_joint[i] = 0.0f;
        cyan_hand_cmd_. mode   [i] = 0.0f;
        cyan_hand_data_.q    [i] = 0.0f;
        cyan_hand_data_.qd   [i] = 0.0f;
        cyan_hand_data_.tauIq[i] = 0.0f;
        cyan_hand_data_.ext_contact[i]  = 0.0f;
        cyan_hand_mid_data_.q    [i] = 0.0f;
        cyan_hand_mid_data_.qd   [i] = 0.0f;
        cyan_hand_mid_data_.tauIq[i] = 0.0f;
        cyan_hand_mid_data_.ext_contact[i]  = 0.0f;
        cyan_legged_cmd_.tau_ff  [i]    = 0.0f;
        cyan_legged_cmd_.q_des   [i]    = 0.0f;
        cyan_legged_cmd_.qd_des  [i]    = 0.0f;
        cyan_legged_cmd_.kp_joint[i]    = 0.0f;
        cyan_legged_cmd_.kd_joint[i]    = 0.0f;
        cyan_legged_cmd_.toruqe_range_ratio[i]    = 0.0f;
        cyan_legged_data_.q[i]          = 0.0f;
        cyan_legged_data_.qd[i]         = 0.0f;
        cyan_legged_data_.tauIq[i]      = 0.0f;
        cyan_legged_data_.tauCom[i]      = 0.0f;
        cyan_legged_mid_data_.q[i]          = 0.0f;
        cyan_legged_mid_data_.qd[i]         = 0.0f;
        cyan_legged_mid_data_.tauIq[i]      = 0.0f;
        cyan_legged_mid_data_.tauCom[i]      = 0.0f;
    }
    
    for (int i = 0; i < 3; i++)
    {
        cyan_lower_bodyimu_data_.accelerometer[i]    = 0.0f;
        cyan_lower_bodyimu_data_.gyro[i]             = 0.0f;
        cyan_lower_bodyimu_data_.rpy[i]              = 0.0f;
        cyan_lower_bodyimu_data_.quat[i]             = 0.0f;
        cyan_lower_bodyimu_data_.posEst[i]           = 0.0f;
        cyan_lower_bodyimu_data_.velEst[i]           = 0.0f;
        if (i==2) cyan_lower_bodyimu_data_.quat[i+1] = 0.0f;
        cyan_lower_bodyimu_mid_data_.accelerometer[i]    = 0.0f;
        cyan_lower_bodyimu_mid_data_.gyro[i]             = 0.0f;
        cyan_lower_bodyimu_mid_data_.rpy[i]              = 0.0f;
        cyan_lower_bodyimu_mid_data_.quat[i]             = 0.0f;
        cyan_lower_bodyimu_mid_data_.posEst[i]           = 0.0f;
        cyan_lower_bodyimu_mid_data_.velEst[i]           = 0.0f;
        if (i==2) cyan_lower_bodyimu_mid_data_.quat[i+1] = 0.0f;
    }
    for (int i = 0; i < EcsJointNum; i++)
    {
        UserSendcmd_[i].ff_= 0.0;
        UserSendcmd_[i].pos_des_ = 0.0;
        UserSendcmd_[i].kd_ = 0.0;
        UserSendcmd_[i].kp_ = 0.0;
        UserSendcmd_tmp[i].ff_= 0.0;
        UserSendcmd_tmp[i].pos_des_ = 0.0;
        UserSendcmd_tmp[i].kd_ = 0.0;
        UserSendcmd_tmp[i].kp_ = 0.0;
    }
}
void HardwareCFG::handleControllerlegs2RobotLCM(const lcm::ReceiveBuffer* rbuf,
                                      const std::string& chan,
                                      const cyan_legged_cmd_lcmt* msg){
    (void)rbuf;
    (void)chan;
    memcpy(&cyan_legged_cmd_, msg, sizeof(cyan_legged_cmd_lcmt));
}
void HardwareCFG::handleControllerAHW2RobotLCM(const lcm::ReceiveBuffer* rbuf,
                                      const std::string& chan,
                                      const cyan_armwaisthead_cmd_lcmt* msg){
    (void)rbuf;
    (void)chan;
    if (ArmMode::mode != 0) {
        return;
    }
    memcpy(&cyan_armwaisthead_cmd_, msg, sizeof(cyan_armwaisthead_cmd_lcmt));
}
void HardwareCFG::handleControllerAHWLEG2RobotLCM(const lcm::ReceiveBuffer* rbuf,
                                      const std::string& chan,
                                      const cyan_armwaisthead_cmd_lcmt* msg) {
    (void)rbuf;
    (void)chan;
    if (ArmMode::mode != 1) {
        return;
    }
    memcpy(&cyan_armwaisthead_cmd_, msg, sizeof(cyan_armwaisthead_cmd_lcmt));
}
void HardwareCFG::handleController2GamepadLCM(const lcm::ReceiveBuffer* rbuf,
                                      const std::string& chan,
                                      const gamepad_lcmt* msg) {
    (void)rbuf;
    (void)chan;
    if (HandleMode::mode != 1) {
        return;
    }
    gamepad_lcmt gamepad_data{};
    for (int i = 0; i < 2; i++)
    {
        gamepad_data.leftStickAnalog[i] = msg->leftStickAnalog[i];
        gamepad_data.rightStickAnalog[i] = msg->rightStickAnalog[i];
    }
    
    update_xbox_map(gamepad_data.leftStickAnalog, gamepad_data.rightStickAnalog);
    Controller2Robot.publish("gamepad2controller", &gamepad_data);
}

void  HardwareCFG::_thread_imu_run(){
    EKFFilter ekf;
    while (true)
    {
        lordimu_.run();
        // usleep(4000);
        // imu_print_count += 1;
        // std::cout<<"count: " << imu_print_count << std::endl;    
        // if (Normal_print_count%1000==0){ 
        //     std::cout<<"lordimu_.quat: "<<lordimu_.quat.transpose()<<std::endl;
        // }
        Vec3<float> rpy=ori::quatToRPY(lordimu_.quat); //w,x,y,z
        Vec3<float> acc=lordimu_.acc; 
        Vec3<float> ang=lordimu_.gyro; 
        // printf("[rpy] X=%.3f rad/s, Y=%.3f rad/s, Z=%.3f rad/s\n", 
        //     rpy[0], rpy[1], rpy[2]);
    
        // printf("hard [ACC] X=%.3f m/s², Y=%.3f m/s², Z=%.3f m/s²\n", 
        //     acc[0], acc[1], acc[2]);
        
        // change imu install direction to normal direction 
        // Vec3<float> acc_order = {lordimu_.acc[2], lordimu_.acc[1], -lordimu_.acc[0]};
        // Vec3<float> ang_order = {lordimu_.gyro[2], lordimu_.gyro[1], -lordimu_.gyro[0]};

        // our ekf handle
        // std::cout << "EKF:" << ekf.get_quaternion() <<std::endl;
        ekf.update(acc, ang, 0.001f);
        Eigen::Quaternionf q = ekf.get_quaternion();
        Eigen::VectorXf x = ekf.get_state();
        Eigen::Vector4f q_wxyz(q.w(), q.x(), q.y(), q.z());
        Vec3<float> rpy_rad = ori::quatToRPY(q_wxyz); // coeffs: x,y,z,w
        // float temp =  rpy_rad[0];
        // rpy_rad[0] = -rpy_rad[2];
        // rpy_rad[2] = temp;

        // use MahonyAHRS
        // float q[4] = {lordimu_.quat[0], lordimu_.quat[1], lordimu_.quat[2], lordimu_.quat[3]};
        // // float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // initial quaternion
        // MahonyAHRSupdateIMU(q, 
        //                 ang[0], ang[1], ang[2], 
        //                 acc[0], acc[1], acc[2]);

        // Eigen::Vector4f q_wxyz(q[0], q[1], q[2], q[3]);
        // Vec3<float> rpy_rad = ori::quatToRPY(q_wxyz); // coeffs: x,y,z,w
        // std::cout << "q_wxyz:" << q_wxyz <<std::endl;

        mutex_imu_data.lock();
        for (int i = 0; i < 3; i++)
        {
           imu_data.rpy[i] =  rpy_rad[i];
           imu_data.accelerometer[i] =  acc[i];
           imu_data.gyro[i] =  ang[i];
           if (i==2)imu_data.quat[i+1]  = lordimu_.quat[i+1];
           imu_data.quat[i] = lordimu_.quat[i];
        //     if (i==2)imu_data.quat[i+1]  = q_wxyz[i+1];
        //    imu_data.quat[i] = q_wxyz[i];
        }
        // usleep(10000);
        mutex_imu_data.unlock();
        
    }
}
void HardwareCFG::test_leg_control_fulloriginal(){
    test_angle=0.1*sinf(count_iter*0.002);
    mutex_cmd_data.lock();
    for (int i = 0; i < EcsJointNum; i++)
    {
        UserSendcmd_[i].pos_des_ = test_angle;
        UserSendcmd_[i].vel_des_ = 0.1*test_angle;
        UserSendcmd_[i].kp_ = 55;
        UserSendcmd_[i].kd_ = 1;
        UserSendcmd_[i].ff_ = 0.0;
    }
    count_iter++;
    mutex_cmd_data.unlock();
}
// for the hybird ff tau control
void HardwareCFG::joint_leg_control_fulloriginalpd(){
    double tempff=0;
    float kp=0.0f,kd=0.0f;
    float temp_torque_range_ratio=0;
    float temp_torque_range_ratio_arm=0;
    mutex_cmd_data.lock();
    for (int i = 0; i < EcsJointNum; i++)
    {
        if (i<12)
        {
            // left leg and right leg
            UserSendcmd_[i].pos_des_ = cyan_legged_cmd_.q_des[i]; 
            UserSendcmd_[i].vel_des_ = cyan_legged_cmd_.qd_des[i];
            UserSendcmd_[i].kp_ = cyan_legged_cmd_.kp_joint[i];
            UserSendcmd_[i].kd_ = cyan_legged_cmd_.kd_joint[i];
            UserSendcmd_[i].ff_ = cyan_legged_cmd_.tau_ff[i];
        }else{
            // if (i<24)
            // {
                // arm 12 joints
                UserSendcmd_[i].pos_des_ = cyan_armwaisthead_cmd_.q_des[i-LegJointNUM]; 
                UserSendcmd_[i].vel_des_ = cyan_armwaisthead_cmd_.qd_des[i-LegJointNUM];
                UserSendcmd_[i].kp_ = cyan_armwaisthead_cmd_.kp_joint[i-LegJointNUM];
                UserSendcmd_[i].kd_ = cyan_armwaisthead_cmd_.kd_joint[i-LegJointNUM];
                UserSendcmd_[i].ff_ = cyan_armwaisthead_cmd_.tau_ff[i-LegJointNUM];
            // }else if(i==24){
            //     // waist,yaw,roll,pitch
            //     UserSendcmd_[i].pos_des_ = cyan_armwaisthead_cmd_.q_des[i]; 
            //     UserSendcmd_[i].vel_des_ = cyan_armwaisthead_cmd_.qd_des[i];
            //     UserSendcmd_[i].kp_ = cyan_armwaisthead_cmd_.kp_joint[i];
            //     UserSendcmd_[i].kd_ = cyan_armwaisthead_cmd_.kd_joint[i];
            //     UserSendcmd_[i].ff_ = cyan_armwaisthead_cmd_.tau_ff[i];
            // }

            if (i > LegJointNUM + 12) {
                UserSendcmd_[i].pos_des_ = 0.0;
                UserSendcmd_[i].vel_des_ = 0.0;
                UserSendcmd_[i].kp_ = 0.0;
                UserSendcmd_[i].kd_ = 0.0;
                
                if (use_pd_torque_range_ratio_from_headfile_==0)
                {
                    kp = Motor_KP[i];
                    kd = Motor_KD[i];
                    
                }else if(use_pd_torque_range_ratio_from_headfile_==1){
                    kp = cyan_armwaisthead_cmd_.kp_joint[i-LegJointNUM];
                    kd = cyan_armwaisthead_cmd_.kd_joint[i-LegJointNUM];
                    temp_torque_range_ratio_arm=torque_range_ratio_;

                }else if(use_pd_torque_range_ratio_from_headfile_==2){
                    kp = cyan_armwaisthead_cmd_.kp_joint[i-LegJointNUM];
                    kd = cyan_armwaisthead_cmd_.kd_joint[i-LegJointNUM];
                    temp_torque_range_ratio_arm=cyan_armwaisthead_cmd_.toruqe_range_ratio[i-LegJointNUM];
                }else{
                    kp = cyan_armwaisthead_cmd_.kp_joint[i-LegJointNUM];
                    kd = cyan_armwaisthead_cmd_.kd_joint[i-LegJointNUM];
                    temp_torque_range_ratio_arm=torque_ratio_head[i-LegJointNUM];
                }
                
                tempff = (kp*(cyan_armwaisthead_cmd_.q_des[i-LegJointNUM]-cyan_armwaisthead_mid_data_.q[i-LegJointNUM])+kd*(cyan_armwaisthead_cmd_.qd_des[i-LegJointNUM]-cyan_armwaisthead_mid_data_.qd[i-LegJointNUM]));
                UserSendcmd_[i].ff_ = clampMinMax(tempff,-temp_torque_range_ratio_arm*Motor_TorqueMinMax[i],temp_torque_range_ratio_arm*Motor_TorqueMinMax[i]);
                cyan_armwaisthead_mid_data_.tauCom[i-LegJointNUM] = static_cast<float>(UserSendcmd_[i].ff_);
            }
        }
            UserSendcmd_tmp[i].pos_des_  =UserSendcmd_[i].pos_des_;
            UserSendcmd_tmp[i].vel_des_  =UserSendcmd_[i].vel_des_;
            UserSendcmd_tmp[i].kp_  =UserSendcmd_[i].kp_;
            UserSendcmd_tmp[i].kd_  =UserSendcmd_[i].kd_;
            UserSendcmd_tmp[i].ff_  =UserSendcmd_[i].ff_;
    }
    mutex_cmd_data.unlock();

}
// just for damp control
void HardwareCFG::joint_fulldamp(){
    mutex_cmd_data.lock();
    for (int i = 0; i < EcsJointNum; i++)
    {
        if (i<12)
        {
            // left leg and right leg
            UserSendcmd_[i].pos_des_ = cyan_legged_mid_data_.q[i]; 
            UserSendcmd_[i].vel_des_ = 0.0;
            UserSendcmd_[i].kp_ = 50.0;
            UserSendcmd_[i].kd_ = 4.0;
            UserSendcmd_[i].ff_ = 0.0f;//cyan_legged_cmd_.tau_ff[i];
        }else{
            // if (i<24)
            // {
                // arm 12 joints
                UserSendcmd_[i].pos_des_ =cyan_legged_mid_data_.q[i-LegJointNUM]; 
                UserSendcmd_[i].vel_des_ = 0.0;
                UserSendcmd_[i].kp_ = 50.0;
                UserSendcmd_[i].kd_ = 4.0;
                UserSendcmd_[i].ff_ = 0.0f;//cyan_armwaisthead_cmd_.tau_ff[i];
        }
            UserSendcmd_tmp[i].pos_des_  =UserSendcmd_[i].pos_des_;
            UserSendcmd_tmp[i].vel_des_  =UserSendcmd_[i].vel_des_;
            UserSendcmd_tmp[i].kp_  =UserSendcmd_[i].kp_;
            UserSendcmd_tmp[i].kd_  =UserSendcmd_[i].kd_;
            UserSendcmd_tmp[i].ff_  =UserSendcmd_[i].ff_;
    }
    mutex_cmd_data.unlock();

}
// just for hybrid position control
void HardwareCFG::joint_leg_control_fulloriginal(){
    mutex_cmd_data.lock();
    for (int i = 0; i < EcsJointNum; i++)
    {
        if (i<12)
        {
            // left leg and right leg
            UserSendcmd_[i].pos_des_ = cyan_legged_cmd_.q_des[i]; 
            UserSendcmd_[i].vel_des_ = cyan_legged_cmd_.qd_des[i];
            UserSendcmd_[i].kp_ = cyan_legged_cmd_.kp_joint[i];
            UserSendcmd_[i].kd_ = cyan_legged_cmd_.kd_joint[i];
            UserSendcmd_[i].ff_ = 0.0f;//cyan_legged_cmd_.tau_ff[i];
        }else{
            // if (i<24)
            // {
                // arm 12 joints
                UserSendcmd_[i].pos_des_ = cyan_armwaisthead_cmd_.q_des[i-LegJointNUM]; 
                UserSendcmd_[i].vel_des_ = cyan_armwaisthead_cmd_.qd_des[i-LegJointNUM];
                UserSendcmd_[i].kp_ = cyan_armwaisthead_cmd_.kp_joint[i-LegJointNUM];
                UserSendcmd_[i].kd_ = cyan_armwaisthead_cmd_.kd_joint[i-LegJointNUM];
                UserSendcmd_[i].ff_ = 0.0f;//cyan_armwaisthead_cmd_.tau_ff[i];
            // }else if(i==24){
            //     // waist,yaw,roll,pitch
            //     UserSendcmd_[i].pos_des_ = cyan_armwaisthead_cmd_.q_des[i]; 
            //     UserSendcmd_[i].vel_des_ = cyan_armwaisthead_cmd_.qd_des[i];
            //     UserSendcmd_[i].kp_ = cyan_armwaisthead_cmd_.kp_joint[i];
            //     UserSendcmd_[i].kd_ = cyan_armwaisthead_cmd_.kd_joint[i];
            //     UserSendcmd_[i].ff_ = 0.0f;//cyan_armwaisthead_cmd_.tau_ff[i];
            // }
        }
            UserSendcmd_tmp[i].pos_des_  =UserSendcmd_[i].pos_des_;
            UserSendcmd_tmp[i].vel_des_  =UserSendcmd_[i].vel_des_;
            UserSendcmd_tmp[i].kp_  =UserSendcmd_[i].kp_;
            UserSendcmd_tmp[i].kd_  =UserSendcmd_[i].kd_;
            UserSendcmd_tmp[i].ff_  =UserSendcmd_[i].ff_;
    }
    mutex_cmd_data.unlock();

}
void HardwareCFG::set_torque_range_ratio(int use_pd_torque_range_ratio_from_headfile,float torque_range_ratio){
    use_pd_torque_range_ratio_from_headfile_=use_pd_torque_range_ratio_from_headfile;
    torque_range_ratio_ = torque_range_ratio;

}
void HardwareCFG::joint_leg_control_fulloriginalTorque(){
    double tempff=0;
    float kp=0.0f,kd=0.0f;
    mutex_cmd_data.lock();
    float temp_torque_range_ratio=0;
    float temp_torque_range_ratio_arm=0;
    for (int i = 0; i < LegJointNUM; i++)
    {
        UserSendcmd_[i].pos_des_ = 0.0;//leg_control_fullcommand_lcmd.q_des[i];
        UserSendcmd_[i].vel_des_ = 0.0;//leg_control_fullcommand_lcmd.qd_des[i];
        UserSendcmd_[i].kp_ = 0.0;//55;//leg_control_fullcommand_lcmd.kp_joint[i];
        UserSendcmd_[i].kd_ = 0.0;//1;//leg_control_fullcommand_lcmd.kd_joint[i];

        if (use_pd_torque_range_ratio_from_headfile_==0)
        {
            kp = Motor_KP[i];
            kd = Motor_KD[i];

        }else if(use_pd_torque_range_ratio_from_headfile_==1){
            kp = cyan_legged_cmd_.kp_joint[i];
            kd = cyan_legged_cmd_.kd_joint[i];
            temp_torque_range_ratio=torque_range_ratio_;

        }else if(use_pd_torque_range_ratio_from_headfile_==2){

            kp = cyan_legged_cmd_.kp_joint[i];
            kd = cyan_legged_cmd_.kd_joint[i];
            temp_torque_range_ratio=cyan_legged_cmd_.toruqe_range_ratio[i];
        }
        else{
            kp = cyan_legged_cmd_.kp_joint[i];
            kd = cyan_legged_cmd_.kd_joint[i];
            temp_torque_range_ratio=torque_ratio_head[i];
        }
        
        tempff = (kp*(cyan_legged_cmd_.q_des[i]-cyan_legged_mid_data_.q[i])+kd*(cyan_legged_cmd_.qd_des[i]-cyan_legged_mid_data_.qd[i]));
        UserSendcmd_[i].ff_ = clampMinMax(tempff,-temp_torque_range_ratio*Motor_TorqueMinMax[i],temp_torque_range_ratio*Motor_TorqueMinMax[i]);
        cyan_legged_mid_data_.tauCom[i] = static_cast<float>(UserSendcmd_[i].ff_);
    }
    for (int i = LegJointNUM; i < EcsJointNum; i++)
    {    //12-25 sdk up2 30 joints real robot just 25
        UserSendcmd_[i].pos_des_ = 0.0;//leg_control_fullcommand_lcmd.q_des[i];
        UserSendcmd_[i].vel_des_ = 0.0;//leg_control_fullcommand_lcmd.qd_des[i];
        UserSendcmd_[i].kp_ = 0.0;//55;//leg_control_fullcommand_lcmd.kp_joint[i];
        UserSendcmd_[i].kd_ = 0.0;//1;//leg_control_fullcommand_lcmd.kd_joint[i];
        
        if (use_pd_torque_range_ratio_from_headfile_==0)
        {
            kp = Motor_KP[i];
            kd = Motor_KD[i];
            
        }else if(use_pd_torque_range_ratio_from_headfile_==1){
            kp = cyan_armwaisthead_cmd_.kp_joint[i-LegJointNUM];
            kd = cyan_armwaisthead_cmd_.kd_joint[i-LegJointNUM];
            temp_torque_range_ratio_arm=torque_range_ratio_;

        }else if(use_pd_torque_range_ratio_from_headfile_==2){
            kp = cyan_armwaisthead_cmd_.kp_joint[i-LegJointNUM];
            kd = cyan_armwaisthead_cmd_.kd_joint[i-LegJointNUM];
            temp_torque_range_ratio_arm=cyan_armwaisthead_cmd_.toruqe_range_ratio[i-LegJointNUM];
        }else{
            kp = cyan_armwaisthead_cmd_.kp_joint[i-LegJointNUM];
            kd = cyan_armwaisthead_cmd_.kd_joint[i-LegJointNUM];
            temp_torque_range_ratio_arm=torque_ratio_head[i-LegJointNUM];
        }
        
        tempff = (kp*(cyan_armwaisthead_cmd_.q_des[i-LegJointNUM]-cyan_armwaisthead_mid_data_.q[i-LegJointNUM])+kd*(cyan_armwaisthead_cmd_.qd_des[i-LegJointNUM]-cyan_armwaisthead_mid_data_.qd[i-LegJointNUM]));
        UserSendcmd_[i].ff_ = clampMinMax(tempff,-temp_torque_range_ratio_arm*Motor_TorqueMinMax[i],temp_torque_range_ratio_arm*Motor_TorqueMinMax[i]);
        cyan_armwaisthead_mid_data_.tauCom[i-LegJointNUM] = static_cast<float>(UserSendcmd_[i].ff_);
    }
    for (int i = 0; i < EcsJointNum; i++)
    {
            UserSendcmd_tmp[i].pos_des_  =UserSendcmd_[i].pos_des_;
            UserSendcmd_tmp[i].vel_des_  =UserSendcmd_[i].vel_des_;
            UserSendcmd_tmp[i].kp_  =UserSendcmd_[i].kp_;
            UserSendcmd_tmp[i].kd_  =UserSendcmd_[i].kd_;
            UserSendcmd_tmp[i].ff_  =UserSendcmd_[i].ff_;
    }
    
    
    mutex_cmd_data.unlock();
}

// hybrid mode for leg and arm, leg uses torque mode and arm uses position mode
void HardwareCFG::joint_leg_control_fulloriginalHybrid(){
    double tempff=0;
    float kp=0.0f,kd=0.0f;
    mutex_cmd_data.lock();
    float temp_torque_range_ratio=0;
    float temp_torque_range_ratio_arm=0;
    for (int i = 0; i < LegJointNUM; i++)
    {
        UserSendcmd_[i].pos_des_ = 0.0;//leg_control_fullcommand_lcmd.q_des[i];
        UserSendcmd_[i].vel_des_ = 0.0;//leg_control_fullcommand_lcmd.qd_des[i];
        UserSendcmd_[i].kp_ = 0.0;//55;//leg_control_fullcommand_lcmd.kp_joint[i];
        UserSendcmd_[i].kd_ = 0.0;//1;//leg_control_fullcommand_lcmd.kd_joint[i];

        if (use_pd_torque_range_ratio_from_headfile_==0)
        {
            kp = Motor_KP[i];
            kd = Motor_KD[i];

        }else if(use_pd_torque_range_ratio_from_headfile_==1){
            kp = cyan_legged_cmd_.kp_joint[i];
            kd = cyan_legged_cmd_.kd_joint[i];
            temp_torque_range_ratio=torque_range_ratio_;

        }else{
            kp = cyan_legged_cmd_.kp_joint[i];
            kd = cyan_legged_cmd_.kd_joint[i];
            temp_torque_range_ratio=cyan_legged_cmd_.toruqe_range_ratio[i];
        }
        tempff = (kp*(cyan_legged_cmd_.q_des[i]-cyan_legged_mid_data_.q[i])+kd*(cyan_legged_cmd_.qd_des[i]-cyan_legged_mid_data_.qd[i]));
        UserSendcmd_[i].ff_ = clampMinMax(tempff,-temp_torque_range_ratio*Motor_TorqueMinMax[i],temp_torque_range_ratio*Motor_TorqueMinMax[i]);
        cyan_legged_mid_data_.tauCom[i] = static_cast<float>(UserSendcmd_[i].ff_);
    }
    for (int i = LegJointNUM; i < EcsJointNum; i++)
    {    //12-25 sdk up2 30 joints real robot just 25
        UserSendcmd_[i].pos_des_ = cyan_armwaisthead_cmd_.q_des[i-LegJointNUM]; 
        UserSendcmd_[i].vel_des_ = cyan_armwaisthead_cmd_.qd_des[i-LegJointNUM];
        UserSendcmd_[i].kp_ = cyan_armwaisthead_cmd_.kp_joint[i-LegJointNUM];
        UserSendcmd_[i].kd_ = cyan_armwaisthead_cmd_.kd_joint[i-LegJointNUM];
        UserSendcmd_[i].ff_ = 0.0f;//cyan_armwaisthead_cmd_.tau_ff[i];
        // cyan_armwaisthead_mid_data_.tauCom[i-LegJointNUM] = static_cast<float>(UserSendcmd_[i].ff_);
        // std::cout<<"*****************************"<<std::endl;
    }
    for (int i = 0; i < EcsJointNum; i++)
    {
            UserSendcmd_tmp[i].pos_des_  =UserSendcmd_[i].pos_des_;
            UserSendcmd_tmp[i].vel_des_  =UserSendcmd_[i].vel_des_;
            UserSendcmd_tmp[i].kp_  =UserSendcmd_[i].kp_;
            UserSendcmd_tmp[i].kd_  =UserSendcmd_[i].kd_;
            UserSendcmd_tmp[i].ff_  =UserSendcmd_[i].ff_;
    }
    
    
    mutex_cmd_data.unlock();
}

void HardwareCFG::joint_leg_control_fulloriginalHybridpd(){
    double tempff=0;
    float kp=0.0f,kd=0.0f;
    mutex_cmd_data.lock();
    float temp_torque_range_ratio=0;
    float temp_torque_range_ratio_arm=0;
    for (int i = 0; i < LegJointNUM; i++)
    {
        UserSendcmd_[i].pos_des_ = 0.0;//leg_control_fullcommand_lcmd.q_des[i];
        UserSendcmd_[i].vel_des_ = 0.0;//leg_control_fullcommand_lcmd.qd_des[i];
        UserSendcmd_[i].kp_ = 0.0;//55;//leg_control_fullcommand_lcmd.kp_joint[i];
        UserSendcmd_[i].kd_ = 0.0;//1;//leg_control_fullcommand_lcmd.kd_joint[i];

        if (use_pd_torque_range_ratio_from_headfile_==0)
        {
            kp = Motor_KP[i];
            kd = Motor_KD[i];

        }else if(use_pd_torque_range_ratio_from_headfile_==1){
            kp = cyan_legged_cmd_.kp_joint[i];
            kd = cyan_legged_cmd_.kd_joint[i];
            temp_torque_range_ratio=torque_range_ratio_;

        }else if(use_pd_torque_range_ratio_from_headfile_==2){

            kp = cyan_legged_cmd_.kp_joint[i];
            kd = cyan_legged_cmd_.kd_joint[i];
            temp_torque_range_ratio=cyan_legged_cmd_.toruqe_range_ratio[i];
        }
        else{
            kp = cyan_legged_cmd_.kp_joint[i];
            kd = cyan_legged_cmd_.kd_joint[i];
            temp_torque_range_ratio=torque_ratio_head[i];
        }
        
        tempff = (kp*(cyan_legged_cmd_.q_des[i]-cyan_legged_mid_data_.q[i])+kd*(cyan_legged_cmd_.qd_des[i]-cyan_legged_mid_data_.qd[i]));
        UserSendcmd_[i].ff_ = clampMinMax(tempff,-temp_torque_range_ratio*Motor_TorqueMinMax[i],temp_torque_range_ratio*Motor_TorqueMinMax[i]);
        cyan_legged_mid_data_.tauCom[i] = static_cast<float>(UserSendcmd_[i].ff_);
    }
    for (int i = LegJointNUM; i < EcsJointNum; i++)
    {    //12-25 sdk up2 30 joints real robot just 25
        UserSendcmd_[i].pos_des_ = cyan_armwaisthead_cmd_.q_des[i-LegJointNUM];
        UserSendcmd_[i].vel_des_ = cyan_armwaisthead_cmd_.qd_des[i-LegJointNUM];
        UserSendcmd_[i].kp_ = cyan_armwaisthead_cmd_.kp_joint[i-LegJointNUM];
        UserSendcmd_[i].kd_ = cyan_armwaisthead_cmd_.kd_joint[i-LegJointNUM];
        UserSendcmd_[i].ff_ = cyan_armwaisthead_cmd_.tau_ff[i-LegJointNUM];

        if (i > LegJointNUM + 12) {
            UserSendcmd_[i].pos_des_ = 0.0;
            UserSendcmd_[i].vel_des_ = 0.0;
            UserSendcmd_[i].kp_ = 0.0;
            UserSendcmd_[i].kd_ = 0.0;
            
            if (use_pd_torque_range_ratio_from_headfile_==0)
            {
                kp = Motor_KP[i];
                kd = Motor_KD[i];
                
            }else if(use_pd_torque_range_ratio_from_headfile_==1){
                kp = cyan_armwaisthead_cmd_.kp_joint[i-LegJointNUM];
                kd = cyan_armwaisthead_cmd_.kd_joint[i-LegJointNUM];
                temp_torque_range_ratio_arm=torque_range_ratio_;

            }else if(use_pd_torque_range_ratio_from_headfile_==2){
                kp = cyan_armwaisthead_cmd_.kp_joint[i-LegJointNUM];
                kd = cyan_armwaisthead_cmd_.kd_joint[i-LegJointNUM];
                temp_torque_range_ratio_arm=cyan_armwaisthead_cmd_.toruqe_range_ratio[i-LegJointNUM];
            }else{
                kp = cyan_armwaisthead_cmd_.kp_joint[i-LegJointNUM];
                kd = cyan_armwaisthead_cmd_.kd_joint[i-LegJointNUM];
                temp_torque_range_ratio_arm=torque_ratio_head[i-LegJointNUM];
            }
            
            tempff = (kp*(cyan_armwaisthead_cmd_.q_des[i-LegJointNUM]-cyan_armwaisthead_mid_data_.q[i-LegJointNUM])+kd*(cyan_armwaisthead_cmd_.qd_des[i-LegJointNUM]-cyan_armwaisthead_mid_data_.qd[i-LegJointNUM]));
            UserSendcmd_[i].ff_ = clampMinMax(tempff,-temp_torque_range_ratio_arm*Motor_TorqueMinMax[i],temp_torque_range_ratio_arm*Motor_TorqueMinMax[i]);
            cyan_armwaisthead_mid_data_.tauCom[i-LegJointNUM] = static_cast<float>(UserSendcmd_[i].ff_);
        }
    }
    for (int i = 0; i < EcsJointNum; i++)
    {
            UserSendcmd_tmp[i].pos_des_  =UserSendcmd_[i].pos_des_;
            UserSendcmd_tmp[i].vel_des_  =UserSendcmd_[i].vel_des_;
            UserSendcmd_tmp[i].kp_  =UserSendcmd_[i].kp_;
            UserSendcmd_tmp[i].kd_  =UserSendcmd_[i].kd_;
            UserSendcmd_tmp[i].ff_  =UserSendcmd_[i].ff_;
    }
    
    
    mutex_cmd_data.unlock();
}

void HardwareCFG::handle_break() {
    double tempff = 0.0;
    float kp=0.0f, kd=0.0f;
    float temp_torque_range_ratio = 0.0f;
    double qd = 0.0;
    double dqd = 0.0;

    auto point = breakHandler->getData();
    mutex_real_data.lock();
    for (int i = 0; i < LegJointNUM; i++)
    {
        qd = point.at(i);
        dqd = 0.0;

        UserSendcmd_[i].pos_des_ = qd;
        UserSendcmd_[i].vel_des_ = 10.0;
        UserSendcmd_[i].kp_ = 0.0;
        UserSendcmd_[i].kd_ = 0.0;

        // kp = cyan_legged_cmd_.kp_joint[i];
        // kd = cyan_legged_cmd_.kd_joint[i];
        if (breakHandler->kps_[i] < Motor_KP[i]) {
            breakHandler->kps_[i] += 0.2;
        } else if (breakHandler->kps_[i] > Motor_KP[i]) {
            breakHandler->kps_[i] -= 0.2;
        }
        kp = breakHandler->kps_.at(i);
        if (breakHandler->kds_[i] < Motor_KD[i]) {
            breakHandler->kds_[i] += 0.05;
        } else if (breakHandler->kds_[i] > Motor_KD[i]) {
            breakHandler->kds_[i] -= 0.05;
        }
        kd = breakHandler->kds_.at(i);

        temp_torque_range_ratio = torque_range_ratio_;
        tempff = (kp * (qd - cyan_legged_mid_data_.q[i]) + kd * (dqd - cyan_legged_mid_data_.qd[i]));
        UserSendcmd_[i].ff_ = clampMinMax(tempff, -0.85 * Motor_TorqueMinMax[i], 0.85 * Motor_TorqueMinMax[i]);
        cyan_legged_mid_data_.tauCom[i] = static_cast<float>(UserSendcmd_[i].ff_);
    }
    for (int i = LegJointNUM; i < EcsJointNum; i++)
    {
        qd = point.at(i);
        dqd = 0.0;

        UserSendcmd_[i].pos_des_ = qd;
        UserSendcmd_[i].vel_des_ = 10.0;
        UserSendcmd_[i].kp_ = 0.0;
        UserSendcmd_[i].kd_ = 0.0;

        // kp = cyan_armwaisthead_cmd_.kp_joint[i-LegJointNUM];
        // kd = cyan_armwaisthead_cmd_.kd_joint[i-LegJointNUM];
        if (breakHandler->kps_[i] < Motor_KP[i]) {
            breakHandler->kps_[i] += 0.2;
        } else if (breakHandler->kps_[i] > Motor_KP[i]) {
            breakHandler->kps_[i] -= 0.2;
        }
        kp = breakHandler->kps_.at(i);
        if (breakHandler->kds_[i] < Motor_KD[i]) {
            breakHandler->kds_[i] += 0.05;
        } else if (breakHandler->kds_[i] > Motor_KD[i]) {
            breakHandler->kds_[i] -= 0.05;
        }
        kd = breakHandler->kds_.at(i);

        temp_torque_range_ratio = torque_range_ratio_;
        tempff = (kp * (qd - cyan_armwaisthead_mid_data_.q[i-LegJointNUM]) + kd * (dqd - cyan_armwaisthead_mid_data_.qd[i-LegJointNUM]));
        UserSendcmd_[i].ff_ = clampMinMax(tempff, -0.85 * Motor_TorqueMinMax[i], 0.85 * Motor_TorqueMinMax[i]);
        cyan_armwaisthead_mid_data_.tauCom[i-LegJointNUM] = static_cast<float>(UserSendcmd_[i].ff_);
    }
    mutex_real_data.unlock();
}

void HardwareCFG::check_break() {
    if (rc_control.mode == RC_mode::BREAK) {
        return;
    }

    if ((fabs(cyan_lower_bodyimu_data_.rpy[0]) < 0.785) && (fabs(cyan_lower_bodyimu_data_.rpy[1]) < 0.785)) {
        return;
    }

    if (cyan_lower_bodyimu_data_.rpy[1] >= 0.0) {
        breakHandler = &frontBreakHandler;
    } else {
        breakHandler = &backBreakHandler;
    }

    std::vector<float> point;
    std::vector<float> kps;
    std::vector<float> kds;

    mutex_real_data.lock();
    for (int i = 0; i < LegJointNUM; i++)
    {
        point.push_back(cyan_legged_mid_data_.q[i]);
    }
    for (int i = LegJointNUM; i < EcsJointNum; i++)
    {
        point.push_back(cyan_armwaisthead_mid_data_.q[i-LegJointNUM]);
    }
    mutex_real_data.unlock();

    mutex_cmd_data.lock();
    for (int i = 0; i < LegJointNUM; i++)
    {
        kps.push_back(cyan_legged_cmd_.kp_joint[i]);
        kds.push_back(cyan_legged_cmd_.kd_joint[i]);
    }
    for (int i = LegJointNUM; i < EcsJointNum; i++)
    {
        kps.push_back(cyan_armwaisthead_cmd_.kp_joint[i]);
        kds.push_back(cyan_armwaisthead_cmd_.kd_joint[i]);
    }
    mutex_cmd_data.unlock();

    breakHandler->calculateIndex(point);
    breakHandler->setKps(kps);
    breakHandler->setKds(kds);
    rc_control.mode = RC_mode::BREAK;

}

void HardwareCFG::set_sdk_state(int id) {
    cyan_lowlevelsdk_state_.id = id;
}

void HardwareCFG::_thread_sdk_state_run() {
    Robot2Controller.publish("lowlevelsdk_state", &cyan_lowlevelsdk_state_);
}

void  HardwareCFG::_thread_lcmrec_run(){
    Controller2Robot.handle();
    Controller2Robot.handle();
    Controller2Robot.handle();
    Controller2Robot.handle();
}
void  HardwareCFG::_thread_lcmsen_run(){
    mutex_real_data.lock();
    for (int i = 0; i < LegJointNUM; i++)
    {
        cyan_legged_data_.q[i] =    cyan_legged_mid_data_.q[i];
        cyan_legged_data_.qd[i]=    cyan_legged_mid_data_.qd[i];
        cyan_legged_data_.tauIq[i]= cyan_legged_mid_data_.tauIq[i];
        cyan_legged_data_.tauCom[i] = cyan_legged_mid_data_.tauCom[i];
    }
       for (int i = 0; i <ArmWaistHeadNum; i++)
    {
        //12-24,hand,25 waist, 26-30 zero/*  */
        cyan_armwaisthead_data_.q[i] =    cyan_armwaisthead_mid_data_.q[i];
        cyan_armwaisthead_data_.qd[i]=    cyan_armwaisthead_mid_data_.qd[i];
        cyan_armwaisthead_data_.tauIq[i]= cyan_armwaisthead_mid_data_.tauIq[i];
        if(i<16)cyan_armwaisthead_data_.tauCom[i] = cyan_armwaisthead_mid_data_.tauCom[i];
    }
    
    for (int i = 0; i < 3; i++)
    {
        cyan_lower_bodyimu_data_.accelerometer[i] = imu_data.accelerometer[i];
        cyan_lower_bodyimu_data_.rpy[i] = imu_data.rpy[i];
        cyan_lower_bodyimu_data_.gyro[i] = imu_data.gyro[i];
        cyan_lower_bodyimu_data_.quat[i] = imu_data.quat[i];
        cyan_lower_bodyimu_data_.posEst[i] = 0.0f;
        cyan_lower_bodyimu_data_.velEst[i] = 0.0f;
        if (i==2)cyan_lower_bodyimu_data_.quat[i+1] = imu_data.quat[i+1]; 
    }
    mutex_real_data.unlock();

    Robot2Controller.publish("robot2controller_legs",&cyan_legged_data_);
    Robot2Controller.publish( "robot2controller_ahw",&cyan_armwaisthead_data_);
    Robot2Controller.publish("robot2controller_sensors",&cyan_lower_bodyimu_data_);
    
}
void  HardwareCFG::thread_run_xbox(){
    while (true) {
        // printf("in xbox\n");
        js_complete(port_xbox);
    // if (Normal_print_count<2000){Normal_print_count++;}else{Normal_print_count=0;};
}
    
};

void  HardwareCFG::zero_all_motor_cmd(){
    EtherCAT_Run();
};
// read data from motors
void HardwareCFG::read(){
    // while (true)
    // {
        mutex_real_data.lock();
        EtherCAT_Get_State();
        for (int i = 0; i < EcsJointNum; i++)
        {
            if (i<LegJointNUM)
            {
                cyan_legged_mid_data_.q[i] =     (motorDate_recv[i].pos_ ) * directionMotor_[i]- baseMotor_[i];
                cyan_legged_mid_data_.qd[i]=     motorDate_recv[i].vel_ * directionMotor_[i];
                cyan_legged_mid_data_.tauIq[i]=  motorDate_recv[i].tau_ * directionMotor_[i];//current
            }else{
                cyan_armwaisthead_mid_data_.q[i-LegJointNUM]    = (motorDate_recv[i].pos_ - baseMotor_[i]) * directionMotor_[i];
                cyan_armwaisthead_mid_data_.qd[i-LegJointNUM]   = motorDate_recv[i].vel_ * directionMotor_[i];
                cyan_armwaisthead_mid_data_.tauIq[i-LegJointNUM]= motorDate_recv[i].tau_ * directionMotor_[i];//current
            }
        }
        mutex_real_data.unlock();  
}
void HardwareCFG::GoToNormalMode(){
    cut_all_data_forsafe_flag=false;
}
void  HardwareCFG::test_mode(){
    EtherCAT_Run();
};
void HardwareCFG::setAllMotorToPassive(){
    mutex_cmd_data.lock();
    for (int i = 0; i < EcsJointNum; ++i)
    {
        yksSendcmd_[i].pos_des_ = 0.0;//UserSendcmd_[i].pos_des_ * directionMotor_[i] + baseMotor_[i];
        yksSendcmd_[i].vel_des_ = 0.0;//UserSendcmd_[i].vel_des_ * directionMotor_[i];
        yksSendcmd_[i].kp_ = 0.0;
        yksSendcmd_[i].kd_ = 0.0;
        yksSendcmd_[i].ff_ = 0.0*UserSendcmd_[i].ff_ * directionMotor_[i];
        
    }
    mutex_cmd_data.unlock();
    EtherCAT_Send_Command((YKSMotorData*)yksSendcmd_);
    cut_all_data_forsafe_flag=true;
}

//  write cmd to motors
//  Notes: do not change the data scales from here
void HardwareCFG::write(int cmd_types){
        if (!cut_all_data_forsafe_flag)
        {
            mutex_cmd_data.lock();
            for (int i = 0; i < EcsJointNum; ++i)
            {
                yksSendcmd_[i].pos_des_ = (UserSendcmd_[i].pos_des_+ baseMotor_[i]) * directionMotor_[i];
                yksSendcmd_[i].vel_des_ = UserSendcmd_[i].vel_des_ * directionMotor_[i];
                yksSendcmd_[i].kp_ =      UserSendcmd_[i].kp_;
                yksSendcmd_[i].kd_ =      UserSendcmd_[i].kd_;
                yksSendcmd_[i].ff_ =      UserSendcmd_[i].ff_ * directionMotor_[i];
                
            }
            mutex_cmd_data.unlock();
            // cmd_types=0: torque mode, cmd_types=1: position mode, cmd_types=2: hybrid mode(leg torque, arm position)
            if (cmd_types==0)
            {
                EtherCAT_Send_Command((YKSMotorData*)yksSendcmd_);
            }
            else if(cmd_types == 1)
            {
                EtherCAT_Send_Command_Position((YKSMotorData*)yksSendcmd_);
            }
            else if(cmd_types == 2)
            {
                EtherCAT_Send_Command_Hybrid((YKSMotorData*)yksSendcmd_);
            }
        }
}
}
