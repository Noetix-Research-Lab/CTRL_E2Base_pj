//
// Created by han on 7/18/23.
//

#ifndef BAER_CORE_ETHERCATPARAMETER_H
#define BAER_CORE_ETHERCATPARAMETER_H

#include "yaml_io.h"

struct EthercatOptionsFromYaml {

    std::string net_card;
    int ctr_freq;
    int robot_type;
    std::vector<int> node_motor_num;
    std::vector<double> motor_max_torque_rec;
    std::vector<double> motor_min_torque_rec;
    std::vector<double> motor_max_torque_send;
    std::vector<double> motor_min_torque_send;
    std::vector<double> motor_max_current;
    std::vector<double> motor_min_current;
    std::vector<double> motor_max_velocity;
    std::vector<double> motor_min_velocity;

    std::vector<int> right_arm_motor_type;
    std::vector<int> right_leg_motor_type;
    std::vector<int> left_arm_motor_type;
    std::vector<int> left_leg_motor_type;
    std::vector<int> waist_motor_type;

    std::vector<int> right_leg_motor_dir;
    std::vector<int> left_leg_motor_dir;
    std::vector<int> right_arm_motor_dir;
    std::vector<int> left_arm_motor_dir;

    std::vector<int> waist_motor_dir;
    std::vector<double> bias_motor;
    template<typename Archive>
    void Serialize(Archive* a) {
        a->Visit(DRAKE_NVP(net_card));
        a->Visit(DRAKE_NVP(ctr_freq));
        a->Visit(DRAKE_NVP(robot_type));
        a->Visit(DRAKE_NVP(node_motor_num));
        a->Visit(DRAKE_NVP(motor_max_torque_rec));
        a->Visit(DRAKE_NVP(motor_min_torque_rec));
        a->Visit(DRAKE_NVP(motor_max_torque_send));
        a->Visit(DRAKE_NVP(motor_min_torque_send));
        a->Visit(DRAKE_NVP(motor_max_current));
        a->Visit(DRAKE_NVP(motor_min_current));
        a->Visit(DRAKE_NVP(motor_max_velocity));
        a->Visit(DRAKE_NVP(motor_min_velocity));

        a->Visit(DRAKE_NVP(right_arm_motor_type));
        a->Visit(DRAKE_NVP(right_leg_motor_type));
        a->Visit(DRAKE_NVP(left_arm_motor_type));
        a->Visit(DRAKE_NVP(left_leg_motor_type));
        a->Visit(DRAKE_NVP(waist_motor_type));

        a->Visit(DRAKE_NVP(right_leg_motor_dir));
        a->Visit(DRAKE_NVP(left_leg_motor_dir));
        a->Visit(DRAKE_NVP(right_arm_motor_dir));
        a->Visit(DRAKE_NVP(left_arm_motor_dir));
        a->Visit(DRAKE_NVP(waist_motor_dir));
        a->Visit(DRAKE_NVP(bias_motor));
    }
};

#endif //BAER_CORE_ETHERCATPARAMETER_H
