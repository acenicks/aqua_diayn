# DIAYN Pseudo-code, explanations and hyperparameters descriptions:

## Project file structure:
The project file structure should look like this:


## Main issues:
- For the rollouts, DIAYN relies on deep copies of the environment, assuming these environments are self-containing. Unfortunately, when using ROS and Gazebo, our Aqua simulator environment is but an interface to gazebo interacting through ROS topics. Therefore, the deep copy does not actually copy our environment. The rollouts end up interacting with the same main environment.
- Current hack is to make sure *epoch_length* is a multiple of *max_path_length*.
