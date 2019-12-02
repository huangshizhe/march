cmake_minimum_required(VERSION 2.8.3)
project(march_gain_scheduling)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    march_shared_resources
    rospy
    std_msgs
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
    test/config/gain_list.cfg
)

catkin_package(
    CATKIN_DEPENDS
    march_shared_resources
    rospy
    std_msgs
)

install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    catkin_add_nosetests(test/nosetest/one_step_linear_interpolation_test.py)
    add_rostest(test/launch/test_march_gain_scheduling.launch)
endif()
