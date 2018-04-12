#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from ros_plant import ROSPlant

if __name__ == '__main__':

    rospy.init_node('random_agent', anonymous=True)

    env = ROSPlant()
    env.reset()

    while True:
        amplitudes = np.random.uniform(low=-np.pi, high=np.pi, size=(6))
        leg_offsets = np.random.uniform(low=-np.pi, high=np.pi, size=(6))
        phase_offsets = np.random.uniform(low=-np.pi, high=np.pi, size=(6))

        action = np.hstack((amplitudes, leg_offsets, phase_offsets))
        state, cost, _, info = env.step(action)
