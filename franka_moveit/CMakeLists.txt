cmake_minimum_required(VERSION 2.8.3)
project(franka_moveit)

find_package(catkin REQUIRED COMPONENTS
    panda_moveit_config
)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_python_setup()
catkin_install_python(PROGRAMS scripts/create_demo_planning_scene.py 
                               tests/test_scene_interface.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
