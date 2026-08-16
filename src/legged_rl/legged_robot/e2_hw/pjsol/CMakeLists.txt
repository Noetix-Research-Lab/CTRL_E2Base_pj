cmake_minimum_required(VERSION 3.10)
project(pjsol)

find_package(catkin REQUIRED)

set(PJSOL_SO "${CMAKE_CURRENT_SOURCE_DIR}/lib/libpjsol.so")

# 下游 catkin 包真正用到的只有这里：
#   INCLUDE_DIRS → ${catkin_INCLUDE_DIRS}，于是 #include <pjsol/pjsol.hpp> 能找到
#   LIBRARIES    → ${catkin_LIBRARIES}，于是链接时带上 libpjsol.so
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PJSOL_SO}
)

# 预编译库不会自动进 devel/lib，拷过去供运行时加载
if(DEFINED CATKIN_DEVEL_PREFIX)
    add_custom_target(pjsol_devel_lib ALL
            COMMAND ${CMAKE_COMMAND} -E make_directory "${CATKIN_DEVEL_PREFIX}/lib"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${PJSOL_SO}" "${CATKIN_DEVEL_PREFIX}/lib/libpjsol.so"
            )
endif()

install(DIRECTORY include/pjsol
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        )

install(FILES ${PJSOL_SO}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
        GROUP_READ GROUP_EXECUTE
        WORLD_READ WORLD_EXECUTE
        )
