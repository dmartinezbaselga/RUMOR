#!/bin/bash

killall -9 amcl
killall -9 move_base
killall -9 pasive_agent
# killall -9 publish_positions
killall -9 obstacle_tracker_node
killall -9 scans_merger_node
killall -9 active_agent
killall -9 obstacle_extractor_node
killall -9 map_server 
killall -9 global_planner_dovs
killall -9 active_agent_multi
killall -9 active_agent_move_base
killall -9 active_agent_dqn
 