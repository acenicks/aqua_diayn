#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import gym
import aqua_gym_env
from aquacore.srv import SetGait


def set_gait_flex_sine():
    rospy.wait_for_service('/aqua/set_gait')
    try:
        resp = rospy.ServiceProxy('/aqua/set_gait', SetGait)
        resp('flexible-sine')
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == '__main__':

    rospy.init_node('random_agent', anonymous=True)
    set_gait_flex_sine()

    env = gym.envs.make('Aqua-v0')
    env.reset()

    while True:
        amplitudes = np.random.uniform(low=-np.pi, high=np.pi, size=(6))
        leg_offsets = np.random.uniform(low=-np.pi, high=np.pi, size=(6))
        phase_offsets = np.random.uniform(low=-np.pi, high=np.pi, size=(6))

        action = np.hstack((amplitudes, leg_offsets, phase_offsets))
        state, cost, _, info = env.step(action)
