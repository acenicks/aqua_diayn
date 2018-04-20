# Project
# Authors
- Andrew Holliday (260604560 | ahollid@cim.mcgill.ca)
- Karim Koreitem (260460964 | karim.koreitem@mail.mcgill.ca)
- Nikhil Kakodkar (260578689 | nikhil.kakodkar@mail.mcgill.ca)

## Overview
This project implements a deep reinforcement learning algorithm for discovering a set of useful skills in an unsupervised fashion. The algorithm is based on the paper Diversity is All You Need: Learning Diverse Skills without a Reward Function.

**What:**
Learning 'diverse' skills. Diverse in the sense that the skills are optimized to have high entropy in the state space. So better exploration.

**Why:**
1. Use these skills as pretraining mechnism for further learning particular tasks.
2. May be use these skills as a curriculum for achieving complex tasks
3. We also use this method to learn sets of skills over various actuator failure modes

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
|   |   ├── rllab
|   |   ├── sac
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
3 Install missing dependencies (for both sac and ros) using pip by following the "missing_packages.txt" *AFTER SOURCING YOUR FRESH NEW ENVIRONMENT WITH THE COMMAND ABOVE*.
