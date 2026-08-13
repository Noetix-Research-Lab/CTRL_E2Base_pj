#!/usr/bin/env bash

export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export ROBOT_TYPE=e2
export ROBOT_MODEL=e2_23dof
export RECORD_DATA=${RECORD_DATA:-false}
export TELEOP_TYPE=${TELEOP_TYPE:-joy}
source ./devel/setup.bash

RL_CONTROLLERS_PATH=$(rospack find rl_controllers)
export POLICY_FILE=${POLICY_FILE:-${RL_CONTROLLERS_PATH}/policy/e2/policyWalk_e2_15w.onnx}
export POLICY_FILE_DANCE=${POLICY_FILE_DANCE:-${RL_CONTROLLERS_PATH}/policy/e2/policyDown_e2_10w.onnx}
export MOTION_FILE=${MOTION_FILE:-${RL_CONTROLLERS_PATH}/motion_files/e2/e2_down.json}
export POLICY_FILE_DOWN=${POLICY_FILE_DOWN:-${RL_CONTROLLERS_PATH}/policy/e2/policyDown_e2_10w.onnx}
export POLICY_FILE_UP=${POLICY_FILE_UP:-${RL_CONTROLLERS_PATH}/policy/e2/policyUp_e2_10w.onnx}
export MOTION_FILE_DOWN=${MOTION_FILE_DOWN:-${RL_CONTROLLERS_PATH}/motion_files/e2/e2_down.json}
export MOTION_FILE_UP=${MOTION_FILE_UP:-${RL_CONTROLLERS_PATH}/motion_files/e2/e2_up.json}
export SBUS_CONFIG_FILE=${SBUS_CONFIG_FILE:-${RL_CONTROLLERS_PATH}/config/sbus_wfly.yaml}

roslaunch rl_controllers ac_start.launch \
  robot_type:=${ROBOT_TYPE} \
  robot_model:=${ROBOT_MODEL} \
  record_data:=${RECORD_DATA} \
  teleop_type:=${TELEOP_TYPE} \
  sbus_config_file:=${SBUS_CONFIG_FILE} \
  policy_file:=${POLICY_FILE} \
  policy_file_dance:=${POLICY_FILE_DANCE} \
  motion_file:=${MOTION_FILE} \
  policy_file_down:=${POLICY_FILE_DOWN} \
  policy_file_up:=${POLICY_FILE_UP} \
  motion_file_down:=${MOTION_FILE_DOWN} \
  motion_file_up:=${MOTION_FILE_UP}
