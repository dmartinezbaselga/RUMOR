#!/bin/bash

[ -n "$1" ] && [ "$1" -eq "$1" ] 2>/dev/null
if [ $? -ne 0 ]; then
   echo "Usage: $0 number_episodes [-r n_pasives n_actives | -s scenario | -l | -qTable <file> | -t | -a <alpha> | -g <gamma> | -e <epsilon> | -d <epsilon_discount> ]"
   exit -1
fi

g++ start_agents.cpp -o start_agents -Wall -O2 -Werror -pedantic -Werror  -std=c++14
# g++ create_scenario.cpp ../include/rl_dovs/graficas.cpp -o create_scenario -Wall -O2 -Wextra -pedantic -Werror  -std=c++14 -I/usr/local/dislin -ldislin -pthread

# ./start_agents $@ -training_world $i


main_agent_name=active_agent_multi
pidDQN=0
./start_agents $@ 0
if [ $? -ne 0 ]; then
    exit 1
fi

isDQN=false
isMoveBase=false
visualize=true
global=false
no_training=""

for i in "$@" ; do
    if [[ $i == "-dqn" ]] ; then
        isDQN=true
    fi
    if [[ $i == "-r2d2" ]] ; then
        isDQN=true
    fi     
    if [[ $i == "-rllib" ]] ; then
        isDQN=true
    fi
    if [[ $i == "-move_base" ]] ; then
        isMoveBase=true
    fi
    if [[ $i == "-visualize" ]] ; then
        visualize=true
    fi
    if [[ $i == "-global_scenario" ]] ; then
        global=true
    fi
    if [[ $i == "-no_training" ]] ; then
        no_training="--no_training"
    fi
    if [[ $i == "-t" ]]; then
        roslaunch gather_metrics gather_metrics.launch &
        # roslaunch gather_metrics record_scenario.launch &
        sleep 1
    fi
done

if [[ $isDQN == true ]] ; then
    main_agent_name=active_agent_dqn
    # roslaunch rl_dovs dqn_server.launch &
    for ((i=1; i<=$#; i++)); do
        echo "${!i}"
        if [ "${!i}" == "-dqn_weights" ]; then
            # Get the name_of_the_file argument
            file_argument_index=$((i + 1))
            weights_file="${!file_argument_index}"
        fi
        if [ "${!i}" == "-crowdnav" ]; then
            # Get the name_of_the_file argument
            file_argument_index=$((i + 1))
            crowdnav_policy="${!file_argument_index}"
            crowdnav=true
        fi
    done
    if [[ $crowdnav == true ]] ; then
        python crowdnav_server.py --n_episodes $2 --weights_file $weights_file --policy $crowdnav_policy $no_training &
        pidDQN=$!
    else
        python sac_server.py --n_episodes $2 --weights_file $weights_file $no_training &
        pidDQN=$!
    fi
    sleep 10
fi

if [[ $isMoveBase == true ]] ; then
    main_agent_name=active_agent_move_base
fi

launch_file=stage_simulation_agents.launch
flag=-training_world
if [[ $visualize == true ]] ; then
    if [[ $global == true ]]; then
        launch_file=stage_simulation_agents_global.launch
    else
        launch_file=stage.launch
    fi
    flag=""
fi


for i in $(seq $1)
do
    episode_complete=false
    while [ "$episode_complete" = false ]; do
        rm -rf /home/diego/.ros/log
        echo "EPISODIO $i"
        ./start_agents $@ $flag $i

        if [ $? -ne 0 ]; then
            exit 1
        fi

        # gnome-terminal -- roslaunch rl_dovs $launch_file &
        # sleep 2
        # # gnome-terminal -- roslaunch rl_dovs publish_positions.launch &
        # # roslaunch rl_dovs multi_agents.launch
        # gnome-terminal -- roslaunch rl_dovs multi_agents.launch &
        # roslaunch rl_dovs publish_positions.launch
    
        echo "start"
        rm error.txt 2> /dev/null
        touch error.txt
        # roslaunch rl_dovs $launch_file 2>&1 | grep -o -a -m 1 -h "Failed to open file" | head -1 > error.txt &
        # cat error.txt | egrep hola
        # kill -0 $pid
        # rosnode kill -a
        roslaunch rl_dovs $launch_file &
        echo "stage launched"
        pidStage=$!
        sleep 5
        until rosnode list; do
            echo "error stage"
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
            killall -9 $main_agent_name
            echo "rosnode kill-sleep"
            # killall -9 rosmaster
            sleep 20
            killall -9 amcl
            killall -9 pasive_agent
            # killall -9 publish_positions
            killall -9 obstacle_tracker_node
            killall -9 scans_merger_node
            killall -9 active_agent
            killall -9 obstacle_extractor_node
            killall -9 map_server
            killall -9 global_planner_dovs
            killall -9 move_base
            killall -9 $main_agent_name
            roslaunch rl_dovs $launch_file  &
            # roslaunch rl_dovs $launch_file 2>&1 | grep -o -a -m 1 -h "Failed to open file" | head -1 > error.txt &
            echo "stage launched from error"
            pidStage=$!
            sleep 5
        done
        echo "launching publish positions"
        # roslaunch rl_dovs publish_positions.launch > /dev/null &
        # roslaunch rl_dovs publish_positions.launch &
        pidPP=$!
        echo "launching multiagents"
        # ------------Failed to open file------------------
        # roslaunch rl_dovs multi_agents.launch  3>&1 1>&2 2>&3  | grep -o -a -m 1 -h "Request for map failed; trying again..." > error.txt &
        # roslaunch rl_dovs multi_agents.launch  2>&1  | grep -o -a -m 1 -h "Request for map failed; trying again..." > error.txt &
        roslaunch rl_dovs multi_agents.launch &
        if [[ $global == true ]]; then
            sleep 5
            roslaunch rl_dovs global_planner.launch &
        fi
        pidPM=$!
        echo "executing EPISODIO $i"
        sleep 2
        if rosnode list | egrep map; then
            if rostopic list | egrep map; then
                finish_loop=false
                while [ "$finish_loop" = false ]; do
                sleep 2
                    if ! killall -0 $main_agent_name; then
                        echo "episode completed"
                        episode_complete=true
                        finish_loop=true
                        # echo 1
                    elif cat error.txt | egrep "Request for map failed; trying again..."; then
                        echo "Error requesting map"
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
                        killall -9 $main_agent_name
                        episode_complete=false
                        finish_loop=true
                        # echo 2
                    elif cat error.txt | egrep "Failed to open file"; then
                        echo "Error in world"
                        killall -9 amcl
                        killall -9 pasive_agent
                        killall -9 move_base
                        # killall -9 publish_positions
                        killall -9 obstacle_tracker_node
                        killall -9 scans_merger_node
                        killall -9 active_agent
                        killall -9 obstacle_extractor_node
                        killall -9 map_server
                        killall -9 global_planner_dovs
                        killall -9 $main_agent_name
                        episode_complete=false
                        finish_loo
                    elif ! kill -0 $pidStage; then
                        echo "stage error"
                        killall -9 amcl
                        killall -9 pasive_agent
                        # killall -9 publish_positions
                        killall -9 obstacle_tracker_node
                        killall -9 scans_merger_node
                        killall -9 move_base
                        killall -9 active_agent
                        killall -9 obstacle_extractor_node
                        killall -9 map_server
                        killall -9 global_planner_dovs
                        killall -9 $main_agent_name
                        episode_complete=false
                        finish_loop=true
                        # echo 4
                    fi
                    # echo 3
                done
            fi
        fi
        #wait $pidPM
        rosnode kill -a
        echo "killing"
        killall -9 amcl
        killall -2 pasive_agent
        # killall -2 publish_positions
        killall -2 obstacle_tracker_node
        killall -2 scans_merger_node
        killall -2 move_base
        killall -2 active_agent
        killall -2 obstacle_extractor_node
        killall -2 map_server
        killall -2 global_planner_dovs
        killall -2 $main_agent_name
        echo "sleeping rosnode kill"
        sleep 10
        # killall -9 rosmaster
        killall -9 roslaunch
        # killall -9 grep
        echo "waiting for multiagents"
        wait $pidPM
        echo "waiting for stage"
        wait $pidStage
        # echo "waiting for publish positions"
        # wait $pidPP
        if [ "$episode_complete" = false ]; then
            echo "Error, episode did not complete, sleeping"
            sleep 20
            echo "killing again"
            killall -9 amcl
            killall -9 pasive_agent
            killall -9 move_base
            # killall -9 publish_positions
            killall -9 obstacle_tracker_node
            killall -9 scans_merger_node
            killall -9 active_agent
            killall -9 obstacle_extractor_node
            killall -9 map_server
            killall -9 global_planner_dovs
            killall -9 $main_agent_name
            # killall -9 rosmaster
            # killall -9 roslaunch
            # killall -9 rosnode
        fi
    done
done
killall -2 python
while killall -0 python 2> /dev/null; do
:
done