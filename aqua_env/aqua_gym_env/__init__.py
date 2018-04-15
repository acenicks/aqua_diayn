from gym.envs.registration import register

register(
    id='Aqua-v0',
    entry_point='aqua_gym_env.ros_plant:ROSPlant'
)
