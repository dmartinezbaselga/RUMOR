#!/bin/bash

[ -n "$1" ] && [ "$1" -eq "$1" ] 2>/dev/null
if [ $? -ne 0 ]; then
   echo "Usage: $0 number_episodes [-r n_pasives n_actives | -s scenario | -l | -qTable <file> | -t | -a <alpha> | -g <gamma> | -e <epsilon> | -d <epsilon_discount> ]"
   exit -1
fi

g++ test_methods.cpp -o test_methods -Wall -O2 -Werror -pedantic -Werror  -std=c++14
# g++ create_scenario.cpp ../include/rl_dovs/graficas.cpp -o create_scenario -Wall -O2 -Wextra -pedantic -Werror  -std=c++14 -I/usr/local/dislin -ldislin -pthread

# ./test_methods $@ -training_world $i

pidDQN=0
./test_methods $@ 0
if [ $? -ne 0 ]; then
    exit 1
fi

roslaunch rl_dovs dqn_server.launch &
pidDQN=$!
sleep 10
break

launch_file=stage_simulation_agents.launch
flag=-training_world
for i in "$@" ; do
    if [[ $i == "-visualize" ]] ; then
        launch_file=stage_simulation_agents_visualize.launch
        flag=""
        break
    fi
done

for i in $(seq $1)
do
    ./test_methods $@ $flag $i

    if [ $? -ne 0 ]; then
        exit 1
    fi
    for agents_file in "multi_agents_rules.launch" "multi_agents_learning.launch" "multi_agents_dqn.launch"; do
        episode_complete=false
        while [ "$episode_complete" = false ]; do
            rm -rf /home/diego/.ros/log
            echo "EPISODIO $i, file $agents_file"


            # gnome-terminal -- roslaunch rl_dovs $launch_file &
            # sleep 2
            # # gnome-terminal -- roslaunch rl_dovs publish_positions.launch &
            # # roslaunch rl_dovs multi_agents.launch
            # gnome-terminal -- roslaunch rl_dovs multi_agents.launch &
            # roslaunch rl_dovs publish_positions.launch
            echo "start"
            rm error.txt 2> /dev/null
            touch error.txt
            roslaunch rl_dovs $launch_file 2>&1 | grep -o -a -m 1 -h "Failed to open file" | head -1 > error.txt &
            # cat error.txt | egrep hola
            # kill -0 $pid
            # rosnode kill -a
            # roslaunch rl_dovs $launch_file > /dev/null &
            echo "stage launched"
            pidStage=$!
            sleep 5
            until rosnode list; do
                echo "error stage"
                killall -9 amcl
                killall -9 pasive_agent
                killall -9 publish_positions
                killall -9 obstacle_tracker_node
                killall -9 active_agent
                killall -9 obstacle_extractor_node
                killall -9 map_server 
                killall -9 active_agent_multi
                echo "rosnode kill-sleep"
                # killall -9 rosmaster
                sleep 20
                killall -9 amcl
                killall -9 pasive_agent
                killall -9 publish_positions
                killall -9 obstacle_tracker_node
                killall -9 active_agent
                killall -9 obstacle_extractor_node
                killall -9 map_server
                killall -9 active_agent_multi
                # roslaunch rl_dovs $launch_file  > /dev/null &
                roslaunch rl_dovs $launch_file 2>&1 | grep -o -a -m 1 -h "Failed to open file" | head -1 > error.txt &
                echo "stage launched from error"
                pidStage=$!
                sleep 5
            done
            echo "launching publish positions"
            roslaunch rl_dovs publish_positions.launch > /dev/null &
            # roslaunch rl_dovs publish_positions.launch &
            pidPP=$!
            echo "launching multiagents"
            # ------------Failed to open file------------------
            # roslaunch rl_dovs multi_agents.launch  3>&1 1>&2 2>&3  | grep -o -a -m 1 -h "Request for map failed; trying again..." > error.txt &
            roslaunch rl_dovs $agents_file  2>&1  | grep -o -a -m 1 -h "Request for map failed; trying again..." > error.txt &
            # roslaunch rl_dovs multi_agents.launch &
            pidPM=$!
            echo "executing EPISODIO $i"
            if rosnode list | egrep map; then
                if rostopic list | egrep map; then
                    finish_loop=false
                    while [ "$finish_loop" = false ]; do
                        if ! kill -0 $pidPP; then
                            echo "episode completed"
                            episode_complete=true
                            finish_loop=true
                            # echo 1
                        elif cat error.txt | egrep "Request for map failed; trying again..."; then
                            echo "Error requesting map"
                            killall -9 amcl
                            killall -9 pasive_agent
                            killall -9 publish_positions
                            killall -9 obstacle_tracker_node
                            killall -9 active_agent
                            killall -9 obstacle_extractor_node
                            killall -9 map_server
                            killall -9 active_agent_multi
                            episode_complete=false
                            finish_loop=true
                            # echo 2
                        elif cat error.txt | egrep "Failed to open file"; then
                            echo "Error in world"
                            killall -9 amcl
                            killall -9 pasive_agent
                            killall -9 publish_positions
                            killall -9 obstacle_tracker_node
                            killall -9 active_agent
                            killall -9 obstacle_extractor_node
                            killall -9 map_server
                            killall -9 active_agent_multi
                            episode_complete=false
                            finish_loo
                        elif ! kill -0 $pidStage; then
                            echo "stage error"
                            killall -9 amcl
                            killall -9 pasive_agent
                            killall -9 publish_positions
                            killall -9 obstacle_tracker_node
                            killall -9 active_agent
                            killall -9 obstacle_extractor_node
                            killall -9 map_server
                            killall -9 active_agent_multi
                            episode_complete=false
                            finish_loop=true
                            # echo 4
                        fi
                        # echo 3
                    done
                fi
            fi
            #wait $pidPM
            echo "killing"
            killall -9 amcl
            killall -2 pasive_agent
            killall -2 publish_positions
            killall -2 obstacle_tracker_node
            killall -2 active_agent
            killall -2 obstacle_extractor_node
            killall -2 map_server
            killall -2 active_agent_multi
            echo "sleeping rosnode kill"
            sleep 5
            # killall -9 rosmaster
            # killall -9 roslaunch
            # killall -9 grep
            echo "waiting for multiagents"
            wait $pidPM
            echo "waiting for stage"
            wait $pidStage
            echo "waiting for publish positions"
            wait $pidPP
            if [ "$episode_complete" = false ]; then
                echo "Error, episode did not complete, sleeping"
                sleep 20
                echo "killing again"
                killall -9 amcl
                killall -9 pasive_agent
                killall -9 publish_positions
                killall -9 obstacle_tracker_node
                killall -9 active_agent
                killall -9 obstacle_extractor_node
                killall -9 map_server
                killall -9 active_agent_multi
                # killall -9 rosmaster
                # killall -9 roslaunch
                # killall -9 rosnode
            fi
        done
    done
done
killall -2 python3
while killall -0 python3 2> /dev/null; do
:
done