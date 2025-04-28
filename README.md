# RUMOR: Reinforcement learning for Understanding a Model of the Real world for navigation in dynamic environments
This repo provides the code of the work for out paper RUMOR: Reinforcement learning for Understanding a Model of the Real world for navigation in dynamic environments, published in Robotics and Autonomous Systems (RAS) in 2025.
# [Paper](https://www.sciencedirect.com/science/article/pii/S092188902500106X) || [Video](https://youtu.be/owF_Iw3BJPU)

## Abstract
Autonomous navigation in dynamic environments is a complex but essential task for autonomous robots, with recent deep reinforcement learning approaches showing promising results. However, the complexity of the real world makes it infeasible to train agents in every possible scenario configuration. Moreover, existing methods typically overlook factors such as robot kinodynamic constraints, or assume perfect knowledge of the environment. In this work, we present RUMOR, a novel planner for differential-drive robots that uses deep reinforcement learning to navigate in highly dynamic environments. Unlike other end-to-end DRL planners, it uses a descriptive robocentric velocity space model to extract the dynamic environment information, enhancing training effectiveness and scenario interpretation. Additionally, we propose an action space that inherently considers robot kinodynamics and train it in a simulator that reproduces the real world problematic aspects, reducing the gap between the reality and simulation. We extensively compare RUMOR with other state-of-the-art approaches, demonstrating a better performance, and provide a detailed analysis of the results. Finally, we validate RUMOR's performance in real-world settings by deploying it on a ground robot. Our experiments, conducted in crowded scenarios and unseen environments, confirm the algorithm's robustness and transferability.

## Code
To be done...

## Citation
If you use this work in your own research or wish to refer to the paper's results, please use the following BibTeX entries.
```bibtex
@article{martinez2025rumor,
title = {RUMOR: Reinforcement learning for Understanding a Model of the Real world for navigation in dynamic environments},
journal = {Robotics and Autonomous Systems},
pages = {105020},
year = {2025},
issn = {0921-8890},
doi = {https://doi.org/10.1016/j.robot.2025.105020},
author = {Diego Martinez-Baselga and Luis Riazuelo and Luis Montano},
}
