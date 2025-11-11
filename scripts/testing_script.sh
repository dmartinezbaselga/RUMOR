#!/bin/bash
for passives in 6 12; do
    # # # # NR-RUMOR
    ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights sac_dovs_crowdnav_final -orca_agents -sac -use_crowdnav_actions -easy_map -no_training -t
    ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights sac_dovs_crowdnav_collaborative_final -orca_agents -sac -use_crowdnav_actions -easy_map -collaborative -no_training -t
    # # RUMOR
    # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights sac_dovs_final -orca_agents -sac -easy_map -no_training -t -graph
    # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights sac_dovs_collaborative_final -orca_agents -sac -easy_map -no_training -t -collaborative -graph
    # # SARL
    # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights sarl -orca_agents -use_crowdnav_actions -easy_map -no_training -t -crowdnav sarl
    # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights sarl_col -orca_agents -use_crowdnav_actions -easy_map -no_training -t -collaborative -crowdnav sarl
    # # INTR
    # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights intrinsic -orca_agents -use_crowdnav_actions -easy_map -no_training -t -crowdnav tsrl
    # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights intrinsic_col -orca_agents -use_crowdnav_actions -easy_map -no_training -t -collaborative -crowdnav tsrl
    # # # ORCA
    # # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights orca -orca_agents -use_crowdnav_actions -easy_map -no_training -t -crowdnav orca
    # # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights orca -orca_agents -use_crowdnav_actions -easy_map -no_training -t -collaborative -crowdnav orca
    # # # S-DOVS
    # # ./launch.sh 1 500 -r $passives 1 -orca_agents -easy_map -no_training -t
    # # ./launch.sh 1 500 -r $passives 1 -orca_agents -easy_map -no_training -t -collaborative
    # # RGL
    # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights rgl -orca_agents -use_crowdnav_actions -easy_map -no_training -t -crowdnav gcn
    # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights rgl_col -orca_agents -use_crowdnav_actions -easy_map -no_training -t -collaborative -crowdnav gcn
    # # Move base
    # ./launch.sh 1 500 -r $passives 1 -orca_agents -easy_map -no_training -t -move_base

    # Changing tracking/absolute training/execution
    # NR-RUMOR
    # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights sac_dovs_crowdnav_final -orca_agents -sac -use_crowdnav_actions -easy_map -no_training -t -collaborative
    # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights sac_dovs_crowdnav_collaborative_final -orca_agents -sac -use_crowdnav_actions -easy_map -no_training -t
    # RUMOR
    # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights sac_dovs_final -orca_agents -sac -easy_map -no_training -t -graph -collaborative
    # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights sac_dovs_collaborative_final -orca_agents -sac -easy_map -no_training -t -graph
    # SARL
    # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights sarl -orca_agents -use_crowdnav_actions -easy_map -no_training -t -crowdnav sarl -collaborative
    # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights sarl_col -orca_agents -use_crowdnav_actions -easy_map -no_training -t -crowdnav sarl
    # # INTR
    # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights intrinsic -orca_agents -use_crowdnav_actions -easy_map -no_training -t -crowdnav tsrl -collaborative
    # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights intrinsic_col -orca_agents -use_crowdnav_actions -easy_map -no_training -t -crowdnav tsrl
    # # RGL
    # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights rgl -orca_agents -use_crowdnav_actions -easy_map -no_training -t -crowdnav gcn -collaborative
    # ./launch.sh 1 500 -r $passives 1 -dqn -visualize -dqn_weights rgl_col -orca_agents -use_crowdnav_actions -easy_map -no_training -t -crowdnav gcn
done
