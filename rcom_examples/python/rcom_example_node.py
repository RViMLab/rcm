#!/usr/bin/env python

import time
import rospy
from actionlib import SimpleActionClient

from rcom_msgs.msg import rcom, rcomAction, rcomGoal  # package specific messages

if __name__ == '__main__':
    rospy.init_node('moveit_motion_examples')

    action_server = rospy.get_param('action_server')

    # create rcom action client
    action_client = SimpleActionClient(
        ns=action_server,
        ActionSpec=rcomAction
    )

    # wait for server to come up
    rospy.loginfo('Waiting for RCoM action server...')
    action_client.wait_for_server()
    rospy.loginfo('Done.')

    # send sample task
    rcom_goal = rcomGoal()

    rate = rospy.Rate(10)
    duration = 5.
    start = time.time()
    while (time.time() - start < duration):
        rcom_goal.states.task.is_velocity = True
        rcom_goal.states.task.values = [0.1, 0., 0.]  # move along x axis

        rcom_goal.states.p_trocar.is_empty = True

        action_client.send_goal(rcom_goal)
        rate.sleep()
