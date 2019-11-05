cmake_minimum_required(VERSION 2.8.3)
project(jackal_gazebo)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch/jackal_world.launch)

install(DIRECTORY launch Media worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#Executable #1
#add_executable(warehouse_localization_node 
#    src/warehouse_localization_node.cpp 
#    src/warehouse_localization.cpp
#)
#target_link_libraries(warehouse_localization_node  
#    ${catkin_LIBRARIES}
#)