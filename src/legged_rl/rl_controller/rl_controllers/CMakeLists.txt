cmake_minimum_required(VERSION 3.10)
project(rl_controllers)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_common
        dynamic_reconfigure
        controller_interface
        controller_manager_msgs
        geometry_msgs
        hardware_interface
        kdl_parser
        sensor_msgs
        std_msgs
        robot_state_publisher
        tf
        urdf
        )
catkin_python_setup()
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ONNXRUNTIME QUIET libonnxruntime)
if(NOT ONNXRUNTIME_FOUND)
        find_package(onnxruntime REQUIRED)
else()
        # catkin_package expects variables matching the DEPENDS spelling.
        set(onnxruntime_INCLUDE_DIRS ${ONNXRUNTIME_INCLUDE_DIRS})
        set(onnxruntime_LIBRARIES ${ONNXRUNTIME_LIBRARIES})
endif()

catkin_package(
        INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        legged_common
        controller_interface
        controller_manager_msgs
        dynamic_reconfigure
        geometry_msgs
        hardware_interface
        kdl_parser
        sensor_msgs
        std_msgs
        robot_state_publisher
        tf
        urdf
        DEPENDS
        onnxruntime
)

###########
## Build ##
###########
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
        src/core/ControlStateMachine.cpp
        src/core/PolicyRuntimeConfig.cpp
        src/config/PolicyMetadata.cpp
        src/config/PolicyContractValidator.cpp
        src/inference/OnnxPolicyModel.cpp
        src/inference/PolicyInferenceRuntimeState.cpp
        src/policies/DefaultMode.cpp
        src/policies/LieMode.cpp
        src/policies/StandMode.cpp
        src/policies/WalkPolicy.cpp
        src/policies/WalkMode.cpp
        src/policies/DancePolicy.cpp
        src/policies/DanceMode.cpp
        src/policies/DownMode.cpp
        src/policies/UpMode.cpp
        src/RLControllerBase.cpp
        src/AcController.cpp
        src/PdTrajectoryLoader.cpp
        src/PdTrajectoryPlayer.cpp
        src/PdTrajectoryDiagnosticLogger.cpp
        src/utilities.cpp
        )

if(CATKIN_ENABLE_TESTING)
        add_test(NAME remote_control_test
                COMMAND ${CMAKE_COMMAND} -E env
                "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src"
                ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_remote_control.py)
        catkin_add_gtest(control_state_machine_test
                test/control_state_machine_test.cpp
                src/core/ControlStateMachine.cpp)
        if(TARGET control_state_machine_test)
                target_include_directories(control_state_machine_test PRIVATE include)
        endif()
        catkin_add_gtest(policy_runtime_config_test
                test/policy_runtime_config_test.cpp
                src/core/PolicyRuntimeConfig.cpp)
        if(TARGET policy_runtime_config_test)
                target_include_directories(policy_runtime_config_test PRIVATE include)
        endif()
        catkin_add_gtest(policy_playback_clock_test
                test/policy_playback_clock_test.cpp)
        if(TARGET policy_playback_clock_test)
                target_include_directories(policy_playback_clock_test PRIVATE include)
        endif()
        catkin_add_gtest(pd_trajectory_loader_test
                test/pd_trajectory_loader_test.cpp
                src/PdTrajectoryLoader.cpp)
        if(TARGET pd_trajectory_loader_test)
                target_include_directories(pd_trajectory_loader_test PRIVATE include)
                target_compile_definitions(pd_trajectory_loader_test PRIVATE
                        TEST_ARM_WAVE_CSV="${CMAKE_CURRENT_SOURCE_DIR}/trajectories/sim/arm/arm_wave_4s_500Hz.csv")
                target_link_libraries(pd_trajectory_loader_test ${catkin_LIBRARIES})
        endif()
        catkin_add_gtest(realtime_buffers_stress_test
                test/realtime_buffers_stress_test.cpp
                src/inference/PolicyInferenceRuntimeState.cpp)
        if(TARGET realtime_buffers_stress_test)
                target_include_directories(realtime_buffers_stress_test PRIVATE include)
                target_link_libraries(realtime_buffers_stress_test Threads::Threads)
        endif()
endif()

add_executable(policy_latency_benchmark
        tools/policy_latency_benchmark.cpp
        src/inference/OnnxPolicyModel.cpp)
target_include_directories(policy_latency_benchmark PRIVATE include)
target_link_libraries(policy_latency_benchmark onnxruntime)

add_executable(model_lifecycle_stress
        tools/model_lifecycle_stress.cpp
        src/inference/OnnxPolicyModel.cpp)
target_include_directories(model_lifecycle_stress PRIVATE include)
target_link_libraries(model_lifecycle_stress onnxruntime)

if(CATKIN_ENABLE_TESTING)
        catkin_add_gtest(onnx_policy_model_test
                test/onnx_policy_model_test.cpp
                src/inference/OnnxPolicyModel.cpp
                src/config/PolicyMetadata.cpp
                src/config/PolicyContractValidator.cpp)
        if(TARGET onnx_policy_model_test)
                target_include_directories(onnx_policy_model_test PRIVATE include)
                target_compile_definitions(onnx_policy_model_test PRIVATE
                        TEST_POLICY_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/policy/e2/policyWalk_e2_15w.onnx")
                target_link_libraries(onnx_policy_model_test onnxruntime)
        endif()
        catkin_add_gtest(policy_execution_test
                test/policy_execution_test.cpp
                src/inference/OnnxPolicyModel.cpp
                src/config/PolicyMetadata.cpp
                src/policies/WalkPolicy.cpp
                src/policies/DancePolicy.cpp)
        if(TARGET policy_execution_test)
                target_include_directories(policy_execution_test PRIVATE include)
                target_compile_definitions(policy_execution_test PRIVATE
                        TEST_WALK_POLICY_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/policy/e2/policyWalk_e2_15w.onnx"
                        TEST_DANCE_POLICY_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/policy/e1/policyDance_23dof.onnx"
                        TEST_DOWN_POLICY_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/policy/e2/policyDown_e2_10w.onnx"
                        TEST_UP_POLICY_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/policy/e2/policyUp_e2_10w.onnx")
                target_link_libraries(policy_execution_test onnxruntime)
        endif()
endif()

# 链接库
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        onnxruntime
        Threads::Threads
        )

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME} legged_target_trajectories_publisher
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(TARGETS policy_latency_benchmark model_lifecycle_stress
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

catkin_install_python(PROGRAMS
        scripts/detect_cpu_topology.py
        scripts/record_inference_timing.py
        scripts/joy_connection_monitor.py
        scripts/joy_teleop_node.py
        scripts/sbus_teleop.py
        scripts/sbus_teleop_wfly.py
        scripts/sbus_teleop_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES rl_controllers_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
install(DIRECTORY trajectories/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/trajectories
        )
