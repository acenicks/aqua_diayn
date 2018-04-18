#!/usr/bin/env python
# license removed for brevity
# import numpy as np

import rospy
from aqua_diayn.msg import TargetState

# TODO: Add service for changing the target state dynamically
# TODO: Add functionality for proving values in numpy format in the yaml file
# TODO: Source the publish rate from a global param

class TaskManager():

    target_state_topic = '/rl/target_state'

    def __init__(self):
        self.task_description = rospy.get_param("~task_description",[])

        self.target_state_pub = rospy.Publisher(TaskManager.target_state_topic,
                                                TargetState,
                                                queue_size=-1)

        self.rate = rospy.Rate(5)  # 10hz

    def spin(self):
        while not rospy.is_shutdown():
            target_state = TargetState()
            target_state.header.stamp = rospy.Time.now()

            target_state.target_state = self.task_description['target_state']

            self.target_state_pub.publish(target_state)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('task_manager', anonymous=True)

    tm = TaskManager()

    try:
        tm.spin()
    except rospy.ROSInterruptException:
        pass
