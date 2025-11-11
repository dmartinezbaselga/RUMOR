cmake_minimum_required(VERSION 3.0.2)
project(rl_dovs)

set(CMAKE_BUILD_TYPE Release)

add_definitions(-DBOOST_LOG_DYN_LINK=1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs std_srvs genmsg roslib roslaunch std_srvs geometry_msgs sensor_msgs laser_geometry message_generation rviz tf stage_ros_dovs obstacle_detector)
find_package(Armadillo REQUIRED)
find_package(genmsg REQUIRED)
find_package(CGAL)
add_library(dislin SHARED IMPORTED)
set_target_properties(dislin PROPERTIES
        IMPORTED_LOCATION "/usr/local/dislin/libdislin.so"
        )

find_package(Boost REQUIRED COMPONENTS thread iostreams system filesystem program_options)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -L/usr/include -Wall -Wextra -Weffc++ -ansi -pedantic -g -rdynamic -w -frounding-math")

add_definitions("-std=c++14")

# catkin_python_setup()

add_message_files(FILES CollaborativeInfo.msg FloatArray.msg)
add_service_files(
  FILES
  DQN_action.srv
  SAC_action.srv
  DQN_store.srv
  DQN_restart_parameters.srv
  DQN_gym.srv
  SAC_gym.srv
  DQN_gym_lidar.srv
  DQN_gym_reset.srv
  SAC_gym_reset.srv
  DQN_gym_reset_lidar.srv
  DQN_gym_reset_crowdnav.srv
  DQN_gym_crowdnav.srv
  DQN_action_rules.srv
  add_obstacle_map.srv
  request_goal.srv
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs obstacle_detector)

# set(RL_DOVS_LINK_LIBRARIES "${Boost_LIBRARIES} CGAL dislin gmp mpfr Xm ${catkin_LIBRARIES}")

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES scans_merger_dovs obstacle_extractor_dovs obstacle_tracker_dovs obstacle_publisher ${PROJECT_NAME}_gui
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs tf laser_geometry message_runtime rviz 
)

# set(catkin_EXPORTED_TARGETS ${catkin_EXPORTED_TARGETS} rl_dovs_generate_messages_cpp)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} /usr/local/dislin  ${Boost_INCLUDE_DIRS} ${INCLUDE_DIR} ${ARMADILLO_INCLUDE_DIRS})
#link_directories(include/rl_dovs/)

add_library(agent include/rl_dovs/Agent.cpp)
add_library(aagent include/rl_dovs/ActiveAgent.cpp)
add_library(astar include/rl_dovs/astar.cpp)
add_library(calcul include/rl_dovs/calcul.cpp)
add_library(cagent include/rl_dovs/CircularAgent.cpp)
add_library(cdens include/rl_dovs/ComputeDensityVS.cpp)
add_library(dovs include/rl_dovs/dovs.cpp)
add_library(dovts include/rl_dovs/dovts.cpp)
add_library(cgal include/rl_dovs/GeometryCGAL.cpp)
add_library(lagent include/rl_dovs/LinearAgent.cpp)
add_library(pagent include/rl_dovs/PassiveAgent.cpp)
add_library(sagent include/rl_dovs/StaticAgent.cpp)
add_library(strategies include/rl_dovs/Strategies.cpp)
add_library(tagent include/rl_dovs/TrajectoryAgent.cpp)
add_library(graficas include/rl_dovs/graficas.cpp)
add_library(vs include/rl_dovs/VS.cpp)
add_library(utilidades include/rl_dovs/utilidades.cpp)
add_library(aagentclass src/active_agent.cpp)


add_executable(example_agent src/example_agent.cpp)
add_dependencies(example_agent rl_dovs_generate_messages_cpp)
target_link_libraries(example_agent ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(fake_amcl src/fake_amcl.cpp)
add_dependencies(fake_amcl rl_dovs_generate_messages_cpp)
target_link_libraries(fake_amcl ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(global_planner_dovs src/global_planner_dovs.cpp)
add_dependencies(global_planner_dovs rl_dovs_generate_messages_cpp)
target_link_libraries(global_planner_dovs ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pasive_agent src/pasive_agent.cpp)
add_dependencies(pasive_agent rl_dovs_generate_messages_cpp)
target_link_libraries(pasive_agent ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(publish_positions src/publish_positions.cpp)
add_dependencies(publish_positions rl_dovs_generate_messages_cpp)
target_link_libraries(publish_positions ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(scenario_tracker src/scenario_tracker.cpp)
add_dependencies(scenario_tracker rl_dovs_generate_messages_cpp)
target_link_libraries(scenario_tracker ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(active_agent_multi src/active_agent_multi.cpp)
add_dependencies(active_agent_multi rl_dovs_generate_messages_cpp)
target_link_libraries(active_agent_multi ${catkin_LIBRARIES} aagentclass ${Boost_LIBRARIES}  aagent agent lagent sagent strategies tagent vs graficas cagent cdens dovs astar pagent dovts utilidades cgal calcul  
        CGAL
        dislin)

add_executable(active_agent_move_base src/active_agent_move_base.cpp)
add_dependencies(active_agent_move_base rl_dovs_generate_messages_cpp)
target_link_libraries(active_agent_move_base ${catkin_LIBRARIES} aagentclass ${Boost_LIBRARIES}  aagent agent lagent sagent strategies tagent vs graficas cagent cdens dovs astar pagent dovts utilidades cgal calcul  
        CGAL
        dislin)

add_executable(active_agent_dqn src/active_agent_dqn.cpp)
add_dependencies(active_agent_dqn rl_dovs_generate_messages_cpp)
target_link_libraries(active_agent_dqn ${catkin_LIBRARIES} aagentclass ${Boost_LIBRARIES} aagent agent lagent sagent strategies tagent vs graficas cagent cdens dovs astar pagent dovts utilidades cgal calcul  
        CGAL
        dislin)

add_executable(active_agent_single src/active_agent_single.cpp)
add_dependencies(active_agent_single rl_dovs_generate_messages_cpp)
target_link_libraries(active_agent_single ${catkin_LIBRARIES} aagentclass ${Boost_LIBRARIES}  aagent agent lagent sagent strategies tagent vs graficas cagent cdens dovs astar pagent dovts utilidades cgal calcul  
        CGAL
        dislin)


# catkin_install_python(PROGRAMS scripts/dqn_server.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

catkin_install_python(PROGRAMS scripts/acme_rl_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# catkin_install_python(PROGRAMS scripts/rllib_server.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

catkin_install_python(PROGRAMS scripts/StageGym.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/StageGymSAC.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# catkin_install_python(PROGRAMS scripts/stable_baselines_server.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )