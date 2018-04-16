# Project
# Authors
- Andrew Holliday (260604560 | )
- Karim Koreitem (260460964 | karim.koreitem@mail.mcgill.ca)
- Nikhil Kakodkar (260578689 | nikhil.kakodkar@mail.mcgill.ca)

## Project file structure:
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


## Environment setup:


1. Install sac_py2 conda environment:
```
cd PATH_TO_ROS_WS/src/aqua_diayn/sac
conda env create -f environment_py2.yml
```

2. Install missing dependencies (for both sac and ros) using pip by following the "missing_packages.txt".

3. Setup your environment:

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
