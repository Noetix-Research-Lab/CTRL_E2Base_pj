# 指定CMake的最低版本要求为3.10
cmake_minimum_required(VERSION 3.10)

# 设置项目名称为legged_e2_hw
project(legged_e2_hw)

SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )

# 启用编译命令导出，这通常用于代码分析工具
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 设置C++标准为C++17
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# 添加子目录，这些子目录包含了第三方库和项目自身的模块
add_subdirectory(third-party/ParamHandler)
add_subdirectory(third-party/inih)
add_subdirectory(common)
add_subdirectory(third-party/SOEM)

# 查找并配置catkin包，指定所需的组件
#当你使用find_package(catkin REQUIRED)时，CMake会查找并配置catkin包。catkin包通常位于ROS工作空间的devel或install目录下，这些目录包含了编译后的库、可执行文件、头文件等。
#COMPONENTS关键字用于指定find_package命令查找的包所依赖的其他组件或库。在这个例子中，roscpp、legged_hw和tf是catkin包的组件，它们是项目构建所必需的库。
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        roslib
        std_msgs
        message_generation
        legged_hw
        tf
        parallel_solve
        )

add_message_files(
        FILES
        MotorFeedbackStamped.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

# 使用catkin_package命令声明catkin包的元信息
catkin_package(
        INCLUDE_DIRS    #用于指定包含头文件的目录，它会包含该目录下和子目录下的所有头文件
        common
        include
        third-party
        user
        CATKIN_DEPENDS   #指定该包所依赖的其他Catkin包
        roscpp
        roslib
        std_msgs
        message_runtime
        legged_hw
        parallel_solve
)

###########
## Build ##
###########

# 包含头文件目录，这些目录包含了项目所需的各种头文件
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${Eigen3_INCLUDE_DIRS} 
        ${parallel_solve_INCLUDE_DIR}
        common
        third-party/inih
        third-party/ParamHandler
        third-party/SOEM/osal
        third-party/SOEM/osal/linux
        third-party/SOEM/oshw/linux
        third-party/SOEM/soem
        /opt/drake/include
        /opt/drake/include/drake/common/yaml
        user/e2-core/FSM_States
        user/e2-core/hardware
)

# 查找并配置Drake库，指定其路径
find_package(drake CONFIG REQUIRED PATHS /opt/drake)

# 使用GLOB_RECURSE收集用户源文件
file(GLOB_RECURSE sources_user               
        "user/e2-core/FSM_States/*.cpp"
        "user/e2-core/hardware/*.cpp"
        "common/*.cpp"
        )

# 声明一个C++可执行文件，指定其源文件
add_executable(legged_e2_hw
        src/legged_e2_hw.cpp
        src/E2HW.cpp
        ${sources_user}
        )
add_dependencies(legged_e2_hw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_library(motor ${SOURCES} 
        src/utilities.cpp)
# 指定可执行文件需要链接的库
target_link_libraries(legged_e2_hw
        ${catkin_LIBRARIES}
        drake::drake
        dynacore_param_handler
        inih
        core
        motor
        ) 

# 设置编译选项，添加调试信息
target_compile_options(legged_e2_hw PRIVATE -g)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(can_error_detector_test test/can_error_detector_test.cpp)
  if(TARGET can_error_detector_test)
    target_include_directories(can_error_detector_test PRIVATE include)
  endif()

  catkin_add_gtest(ethercat_wkc_monitor_test test/ethercat_wkc_monitor_test.cpp)
  if(TARGET ethercat_wkc_monitor_test)
    target_include_directories(ethercat_wkc_monitor_test PRIVATE include)
  endif()

  catkin_add_gtest(pvt_command_safety_test test/pvt_command_safety_test.cpp)
  if(TARGET pvt_command_safety_test)
    target_include_directories(pvt_command_safety_test PRIVATE include)
  endif()

  catkin_add_gtest(ethercat_cycle_snapshot_test test/ethercat_cycle_snapshot_test.cpp)
  if(TARGET ethercat_cycle_snapshot_test)
    target_include_directories(ethercat_cycle_snapshot_test PRIVATE include)
  endif()

  catkin_add_gtest(e2_joint_map_test test/e2_joint_map_test.cpp)
  if(TARGET e2_joint_map_test)
    target_include_directories(e2_joint_map_test PRIVATE include)
  endif()
endif()

#########################
###   CLANG TOOLING   ###
#########################

# 查找并配置cmake_clang_tools，如果找到则启用clang工具链
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " legged_e2_hw)
    add_clang_tooling(
            TARGETS legged_e2_hw
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

# 复制配置文件到构建目录
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config/ethercat_config.yaml DESTINATION
        ${CMAKE_CURRENT_BINARY_DIR}/user/e2-core)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config/config.yaml DESTINATION
        ${CMAKE_CURRENT_BINARY_DIR}/user/e2-core)

#############
## Install ##
#############

# 标记可执行文件和/或库文件以便安装
install(TARGETS legged_e2_hw
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# 标记C++头文件以便安装
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# 标记其他文件以便安装
install(DIRECTORY config launch lib
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
