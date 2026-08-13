catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
chmod +x \
  src/legged_rl/rl_controller/rl_controllers/scripts/sbus_teleop_node.py \
  src/legged_rl/rl_controller/rl_controllers/scripts/sbus_teleop_wfly.py \
  src/legged_rl/rl_controller/rl_controllers/scripts/sbus_teleop.py \
  src/legged_rl/legged_base/legged_common/scripts/generate_urdf.sh
catkin build
