#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rospy
from actionlib.simple_action_client import SimpleActionClient

from rcom_msgs.msg import rcom, rcomActionFeedback

# subscribe to jacobians
# plot manipulability


# plot errors as received from callbacks
class Plotter:
    def __init__(self):
        self.fig, (self.ax1, self.ax2) = plt.subplots(1,2)

        self.lines = [
            self.ax1.plot([], [])[0], self.ax1.plot([], [])[0], self.ax1.plot([], [])[0],  # trocar position dx, dy, dz
            self.ax2.plot([], [])[0], self.ax2.plot([], [])[0], self.ax2.plot([], [])[0],  # task position dx, dy, dz
            # self.ax3.plot([], [])[0]                                                       # task orientation droll
        ]

        self.seq = []
        self.p_trocar_errors = [[], [], []]
        self.task_errors = np.array([])

    def plot_init(self):
        self.ax1.set_xlim(0, 3000)
        self.ax1.set_ylim(-0.02, 0.02)
        self.ax2.set_xlim(0, 3000)
        # self.ax2.set_ylim(-0.02, 0.02)
        # self.ax3.set_xlim(0, 3000)
        # self.ax3.set_ylim(-0.02, 0.02)
        plt.tight_layout()
        return self.lines

    def rcom_action_server_callback(self, msg):
        # log sequence number
        self.seq.append(msg.header.seq)

        # log trocar position errors
        self.p_trocar_errors[0].append(msg.feedback.errors.p_trocar.position.x)
        self.p_trocar_errors[1].append(msg.feedback.errors.p_trocar.position.y)
        self.p_trocar_errors[2].append(msg.feedback.errors.p_trocar.position.z)

        # log task position errors
        task_error = np.asarray(msg.feedback.errors.task.values)
        task_error = np.expand_dims(task_error, 1)
        if self.task_errors.size == 0:
            self.task_errors = task_error
        else:
            self.task_errors = np.concatenate((self.task_errors, task_error), axis=1)

    def jacobian_callback(self, msg):
        pass

    def update_plot(self, frame):
        for idx in range(0,3):
            self.lines[idx].set_data(self.seq, self.p_trocar_errors[idx])
        if self.task_errors.size != 0:
            for idx in range(3,len(self.lines)):
                self.lines[idx].set_data(np.asarray(self.seq), self.task_errors[idx - 3])
        return self.lines


if __name__ == '__main__':

    rospy.init_node('log_node')    
    
    action_server = rospy.get_param('/lbr/action_server')

    plotter = Plotter()

    rospy.Subscriber('/lbr/{}/feedback'.format(action_server), rcomActionFeedback, plotter.rcom_action_server_callback)
    ani = animation.FuncAnimation(plotter.fig, plotter.update_plot, init_func=plotter.plot_init)
    plt.show(block=True)
