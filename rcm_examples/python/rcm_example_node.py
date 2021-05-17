#!/usr/bin/env python

import time
import rospy
from actionlib import SimpleActionClient

from rcm_msgs.msg import rcm, rcmAction, rcmGoal  # package specific messages

if __name__ == '__main__':
    rospy.init_node('moveit_motion_examples')

    action_server = rospy.get_param('action_server')

    # create rcm action client
    action_client = SimpleActionClient(
        ns=action_server,
        ActionSpec=rcmAction
    )

    # wait for server to come up
    rospy.loginfo('Waiting for RCM action server...')
    action_client.wait_for_server()
    rospy.loginfo('Done.')

    # send sample task
    rcm_goal = rcmGoal()

    rate = rospy.Rate(10)
    duration = 5.
    start = time.time()
    while (time.time() - start < duration):
        rcm_goal.states.task.is_velocity = True
        rcm_goal.states.task.values = [0.1, 0., 0.]  # move along x axis

        rcm_goal.states.p_trocar.is_empty = True

        action_client.send_goal(rcm_goal)
        rate.sleep()
