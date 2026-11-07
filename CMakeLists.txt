cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#使用 C++14 标准,并且强制要求编译器支持(不支持就报错,而不是悄悄降级)。
## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror) 
#- `-Wall`：打开**全部常规编译警告**
#- `-Werror`：**所有警告直接当成错误**，编译不通过。这是强约束，用来强制代码规范，不允许遗留警告。
#- `add_definitions`：全局添加编译参数，作用于本包所有源码。

## Find catkin macros and libraries
find_package(catkin REQUIRED #`REQUIRED`：缺失则编译终止。
    COMPONENTS #声明依赖的ROS组件
    roscpp #ROS C++ 客户端库（节点、发布订阅、服务）
    sensor_msgs #ROS 传感器消息包（Imu、Joy、LaserScan 等）
    roslint #ROS 静态代码检查工具（cpplint）
    )

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package( #`catkin_package()` 是 catkin 核心宏，**给别的包看的导出信息**，别的包依赖这个包时会用到这些信息。
    INCLUDE_DIRS
    include
    ## This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
    ${EIGEN3_INCLUDE_DIR}

    LIBRARIES
    ${PROJECT_NAME}_core

    CATKIN_DEPENDS
    roscpp
    sensor_msgs
    
    DEPENDS
    ## find_package(Eigen3) provides a non standard EIGEN3_INCLUDE_DIR instead of Eigen3_INCLUDE_DIRS.
    ## Therefore, the DEPEND does not work as expected and we need to add the directory to the INCLUDE_DIRS
    # Eigen3

    ## Boost is not part of the DEPENDS since it is only used in source files,
    ## Dependees do not depend on Boost when they depend on this package.
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    # Set manually because Eigen sets a non standard INCLUDE DIR
    ${EIGEN3_INCLUDE_DIR}
    # Set because Boost is an internal dependency, not transitive.
    ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_core
    src/Algorithm.cpp
    )
target_compile_features(${PROJECT_NAME}_core INTERFACE cxx_std_11)

## Declare cpp executables
add_executable(${PROJECT_NAME}
    src/${PROJECT_NAME}_node.cpp
    src/RosPackageTemplate.cpp
    )
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_core
    ${catkin_EXPORTED_TARGETS}
    )

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    )

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_core
    ${catkin_LIBRARIES}
    )

target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_core
    ${catkin_LIBRARIES}
    )

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
    TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
    DIRECTORY doc
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if (${CATKIN_ENABLE_TESTING})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(${PROJECT_NAME}-test
      test/test_ros_package_template.cpp
      test/AlgorithmTest.cpp)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
endif ()

##########################
## Static code analysis ##
##########################

roslint_cpp()
