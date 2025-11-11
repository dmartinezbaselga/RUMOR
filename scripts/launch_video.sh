#!/bin/bash

[ -n "$1" ] && [ "$1" -eq "$1" ] 2>/dev/null
if [ $? -ne 0 ]; then
   echo "Usage: $0 number_episodes [-r n_pasives n_actives | -s scenario | -l | -qTable <file> | -t | -a <alpha> | -g <gamma> | -e <epsilon> | -d <epsilon_discount> | -graph | -video | -track_metric ]"
   exit -1
fi

g++ start_agents.cpp -o start_agents -Wall -O2 -Werror -pedantic -Werror  -std=c++14
# g++ create_scenario.cpp ../include/rl_dovs/graficas.cpp -o create_scenario -Wall -O2 -Wextra -pedantic -Werror  -std=c++14 -I/usr/local/dislin -ldislin -pthread

pidDQN=0
./start_agents $@ 0
if [ $? -ne 0 ]; then
    exit 1
fi
for i in "$@" ; do
    if [[ $i == "-dqn" ]] ; then
        roslaunch rl_dovs dqn_server.launch &
        pidDQN=$!
        echo "PID $pidDQN"
        sleep 10
        break
    fi
done

for i in $(seq $1)
do
    episode_complete=false
    while [ "$episode_complete" = false ]; do
        rm -rf /home/diego/.ros/log
        echo "EPISODIO $i"
        ./start_agents $@ $i

        if [ $? -ne 0 ]; then
            exit 1
        fi

        # gnome-terminal -- roslaunch rl_dovs stage_simulation_agents.launch &
        # sleep 2
        # # gnome-terminal -- roslaunch rl_dovs publish_positions.launch &
        # # roslaunch rl_dovs multi_agents.launch
        # gnome-terminal -- roslaunch rl_dovs multi_agents.launch &
        # roslaunch rl_dovs publish_positions.launch
    
        echo "start"
        # roslaunch rl_dovs stage_simulation_agents.launch 2>&1 | grep -o -a -m 1 -h hola | head -1 > error.txt
        # cat error.txt | egrep hola
        # kill -0 $pid
        rm error.txt 2> /dev/null
        touch error.txt
        # rosnode kill -a
        roslaunch rl_dovs stage_simulation_agents_visualize.launch > /dev/null &
        # roslaunch rl_dovs rviz.launch > /dev/null &
        echo "stage launched"
        pidStage=$!
        sleep 5
        until rosnode list; do
            echo "error stage"
            killall -9 amcl
            killall -9 pasive_agent
            # killall -9 publish_positions
            killall -9 obstacle_tracker_node
            killall -9 active_agent
            killall -9 obstacle_extractor_node
            killall -9 map_server
            echo "rosnode kill-sleep"
            # killall -9 rosmaster
            sleep 20
            killall -9 amcl
            killall -9 pasive_agent
            # killall -9 publish_positions
            killall -9 obstacle_tracker_node
            killall -9 active_agent
            killall -9 obstacle_extractor_node
            killall -9 map_server
            roslaunch rl_dovs stage_simulation_agents.launch  > /dev/null &
            echo "stage launched from error"
            pidStage=$!
            sleep 5
        done
        echo "launching publish positions"
        # roslaunch rl_dovs publish_positions.launch > /dev/null &
        # roslaunch rl_dovs publish_positions.launch &
        pidPP=$!
        echo "launching multiagents"
        # roslaunch rl_dovs multi_agents.launch  3>&1 1>&2 2>&3  | grep -o -a -m 1 -h "Request for map failed; trying again..." > error.txt &
        # roslaunch rl_dovs multi_agents.launch  2>&1  | grep -o -a -m 1 -h "Request for map failed; trying again..." > error.txt &
        roslaunch rl_dovs multi_agents.launch &
        pidPM=$!
        # sleep 2
        xdotool search --onlyvisible --name stage_ros windowmove 50 50 windowsize 750 460
        xdotool search --onlyvisible --name rviz windowmove 950 50 windowsize 1150 1200
        xdotool search --onlyvisible --name dislin windowmove 50 520 windowsize 750 540
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
                        episode_complete=false
                        finish_loop=true
                        # echo 2
                    fi
                    xdotool search --onlyvisible --name stage_ros windowmove 50 50 windowsize 750 460
                    xdotool search --onlyvisible --name dislin windowmove 50 520 windowsize 750 540
                    xdotool search --onlyvisible --name rviz windowmove 950 50 windowsize 1150 1200
                    sleep 1
                done
            fi
        fi
        #wait $pidPM
        echo "killing"
        killall -2 amcl
        killall -2 pasive_agent
        # killall -2 publish_positions
        killall -2 obstacle_tracker_node
        killall -2 active_agent
        killall -2 obstacle_extractor_node
        killall -2 map_server
        echo "sleeping rosnode kill"
        sleep 5
        # killall -9 rosmaster
        # killall -9 roslaunch
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
            # killall -9 publish_positions
            killall -9 obstacle_tracker_node
            killall -9 active_agent
            killall -9 obstacle_extractor_node
            killall -9 map_server
            # killall -9 rosmaster
            # killall -9 roslaunch
            # killall -9 rosnode
        fi
    done
done
killall -2 python3
while killall -0 python3 2> /dev/null; do
:
done