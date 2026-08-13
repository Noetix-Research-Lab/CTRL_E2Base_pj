cmake_minimum_required(VERSION 3.0.2)
project(parallel_solve)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  
)

include_directories(
  ankle/include
  waist/include
  ${catkin_INCLUDE_DIRS}
)

add_library(parallel_ankle_cpp SHARED IMPORTED)
set_target_properties(parallel_ankle_cpp PROPERTIES
  IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/ankle/lib/libparallel_ankle_cpp.so
  INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/ankle/include
)

add_library(parallel_waist_cpp SHARED IMPORTED)
set_target_properties(parallel_waist_cpp PROPERTIES
  IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/waist/lib/libparallel_waist_cpp.so
  INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/waist/include
)

catkin_package(
  INCLUDE_DIRS ankle/include waist/include
  LIBRARIES
    ${CMAKE_CURRENT_SOURCE_DIR}/ankle/lib/libparallel_ankle_cpp.so
    ${CMAKE_CURRENT_SOURCE_DIR}/waist/lib/libparallel_waist_cpp.so
  CATKIN_DEPENDS roscpp 
)

install(DIRECTORY ankle/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY waist/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/waist
  FILES_MATCHING PATTERN "*.h"
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/ankle/lib/libparallel_ankle_cpp.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE 
              GROUP_READ GROUP_EXECUTE 
              WORLD_READ WORLD_EXECUTE
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/waist/lib/libparallel_waist_cpp.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
              GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE
)
