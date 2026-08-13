//
// Created by han on 23-2-15.
//

// ControlFSMData.h

#ifndef BAER_ETHERCAT_CONTROLFSMDATA_H
#define BAER_ETHERCAT_CONTROLFSMDATA_H

#include "RemoteUserParameter.h"
#include "memory"
#include "../hardware/Medulla.h"
#include "../../../common/frame.h"

// 定义用于控制有限状态机的数据结构
struct ControlFSMData {
    // 构造函数，接受一个名为 frame_ptr 的 std::shared_ptr<Frame> 参数
    ControlFSMData(const std::shared_ptr<Frame>& frame_ptr){
        // 复制构造，frame_ptr_ 和 frame_ptr 指向相同的内存
        frame_ptr_ = frame_ptr;

        // 获取远程用户参数和机器人类型
        remote_user_para_ = frame_ptr->mutableUserParam();
        robot_type_ = frame_ptr->mutableRobotModel()->robot_type;

        // 初始化控制字
        control_word_ = 0;

        // 初始化关节状态数组
        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 6; ++j) {
                position_des_[i][j] = 0.0;
                velocity_des_[i][j] = 0.0;
                torque_des_[i][j] = 0.0;
                kp_[i][j] = 0.0;
                kd_[i][j] = 0.0;

                position_act_raw_[i][j] = 0.0;
                velocity_act_raw_[i][j] = 0.0;
                current_act_raw_[i][j] = 0.0;
                torque_act_raw_[i][j] = 0.0;
                temperature_raw_[i][j] = 0.0;
                mos_tmp_raw_[i][j] = 0.0;
            }
            
        }
    };

    // 成员变量
    std::shared_ptr<RemoteUserParameter> remote_user_para_;
    std::shared_ptr<Frame> frame_ptr_;
    int robot_type_;
    int state_no_;

    // 用于表示从站的指针
    std::shared_ptr<EthercatSlaveBase> node_1_;
    std::shared_ptr<EthercatSlaveBase> node_2_;
    std::shared_ptr<EthercatSlaveBase> node_3_;
    // std::shared_ptr<EthercatSlaveBase> node_4_;

    // 数据
    int hs_data[5][6];  // 高速数据

    uint16_t control_word_;  // 控制字

    // 关节实际值
    double position_act_raw_[5][6];
    double velocity_act_raw_[5][6];
    double current_act_raw_[5][6];
    double torque_act_raw_[5][6];
    double temperature_raw_[5][6];
    double mos_tmp_raw_[5][6];

    // 关节期望值
    double position_des_[5][6];
    double velocity_des_[5][6];
    double torque_des_[5][6];
    double kp_[5][6];
    double kd_[5][6];

    // 电机设置
    double joint_max_torque_rec_[23];
    double joint_min_torque_rec_[23];
    double joint_max_torque_send_[23];
    double joint_min_torque_send_[23];
    double joint_max_current_[23];
    double joint_min_current_[23];
    double joint_max_velocity_[23];
    double joint_min_velocity_[23];

    // 关节方向
    double joint_dir_[23];

    // 时间
    double time_;
    double begin_time_;
};

#endif //BAER_ETHERCAT_CONTROLFSMDATA_H
