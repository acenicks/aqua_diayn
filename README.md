# Project
# Authors
- Andrew Holliday (260604560 | ahollid@cim.mcgill.ca)
- Karim Koreitem (260460964 | karim.koreitem@mail.mcgill.ca)
- Nikhil Kakodkar (260578689 | nikhil.kakodkar@mail.mcgill.ca)

# NOTE:
This project is built on the source code provided by other researchers, namely:
1. Eysenbach, Benjamin, et al. "Diversity is All You Need: Learning Skills without a Reward Function." arXiv preprint arXiv:1802.06070 (2018).

  Haarnoja, Tuomas, et al. "Soft Actor-Critic: Off-Policy Maximum Entropy Deep Reinforcement Learning with a Stochastic Actor." arXiv preprint arXiv:1801.01290 (2018).

  https://github.com/haarnoja/sac.git

2. Meger, David, et al. "Learning legged swimming gaits from experience." Robotics and Automation (ICRA), 2015 IEEE International Conference on. IEEE, 2015. (This codebase is currently private and internal to the MRL lab at McGill University).

3. Yan Duan, Xi Chen, Rein Houthooft, John Schulman, Pieter Abbeel. "Benchmarking Deep Reinforcement Learning for Continuous Control". Proceedings of the 33rd International Conference on Machine Learning (ICML), 2016.

  https://github.com/rll/rllab.git

## Project file structure
The project file structure should look like this:

```
ros_ws/
└── src
|   ├── CMakeLists.txt
|   ├── aquaautopilot
|   ├── aquacore
|   ├── aquadepth
|   ├── aqua_description
|   |
|   ├── aqua_diayn (this repo)
|   |   ├── aqua_env
|   |   ├── CMakeLists.txt
|   |   ├── launch
|   |   ├── notebooks
|   |   ├── package.xml
|   |   ├── README.md
|   |   ├── rllab           <-- (https://github.com/rll/rllab.git)
|   |   ├── sac             <-- (https://github.com/haarnoja/sac.git)
|   |   ├── scripts
|   |   └── src
|   |
|   ├── aqua_gait
|   ├── aqua_gazebo
|   ├── aquajoy
|   ├── aqualaunch
|   ├── aquapositioning
|   └── robot_learning
└── data (default location)
```


## Environment setup


1. Install sac_py2 conda environment:
```
cd PATH_TO_ROS_WS/src/aqua_diayn/sac
conda env create -f environment_py2.yml
```

2. Setup your environment:

In your ~/.bashrc, define the following function (editing the paths as needed):

```
# ROS

source-rlproject-ws(){

export PYTHONPATH=""
export PROJECT_PATH="PATH_TO_ROS_WS"

source activate sac_py2
export PYTHONPATH="$PROJECT_PATH/src/aqua_diayn/sac:$PYTHONPATH"
export PYTHONPATH="$PROJECT_PATH/src/aqua_diayn/rllab:$PYTHONPATH"
export PYTHONPATH="$PROJECT_PATH/src/aqua_diayn/aqua_env:${PYTHONPATH}"

source /opt/ros/kinetic/setup.bash
source $PROJECT_PATH/devel/setup.bash

}
```
3. In your terminal, source your updated bashrc:
```
source ~/.bashrc
```

4. Source the newly defined environment using our command defined above:
```
source-rlproject-ws
```

5. Install missing dependencies (for both sac and ros) using pip by following the "missing_packages.txt" **AFTER** sourcing the freshly defined environment (i.e after running step 4).
