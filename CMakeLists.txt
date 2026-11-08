cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED  COMPONENTS 
    roscpp 
    #sensor_msgs 
    #roslint  
    controller_interface
    hardware_interface
    pluginlib
    )


#find_package(Eigen3 REQUIRED)


catkin_package( 
    INCLUDE_DIRS
    include
    
    #${EIGEN3_INCLUDE_DIR}

    LIBRARIES
    ${PROJECT_NAME}

    CATKIN_DEPENDS
    roscpp
    controller_interface
    hardware_interface
    pluginlib
    #sensor_msgs
    
    #DEPENDS
    
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
   
)


add_library( ${PROJECT_NAME}  src/hero_chassis_controller.cpp )
target_link_libraries( ${PROJECT_NAME}  ${catkin_LIBRARIES} )

add_dependencies( ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} )




install(TARGETS ${PROJECT_NAME} 
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


install(FILES hero_chassis_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch configure_file
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



#if (${CATKIN_ENABLE_TESTING})
 # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

  #catkin_add_gtest(${PROJECT_NAME}-test
    # test/test_ros_package_template.cpp
    #  test/AlgorithmTest.cpp)
 # target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
#endif ()


#roslint_cpp()
