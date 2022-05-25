#!/usr/bin/env python
import rospy
import math
from gazebo_msgs.msg import ModelState, ModelStates
import numpy as np
from numpy import sin
from numpy import sqrt
from numpy import arange
from pandas import read_csv
from scipy.optimize import curve_fit
from matplotlib import pyplot


x_ser = np.array([])
y_ser = np.array([])
aa = []
bb = []
flag = 0


class landing_mover():
    def __init__(self, model_name="landing_station", x_a=0.3, y_a=0.3, z_a=0.5, x_f=0.6, y_f=0.4, z_f=0.3, x_p=1.37, y_p=0.33, z_p=0):
        self.name = model_name
        self.x_a = x_a
        self.x_f = x_f
        self.x_p = x_p
        self.y_a = y_a
        self.y_f = y_f
        self.y_p = y_p
        self.z_a = z_a
        self.z_f = z_f
        self.z_p = z_p
        self.model_state = ModelState()
        self.model_state.model_name = self.name
        self.model_state.reference_frame = self.name
    
    def update_model_state_pose_sin(self, time):
        self.model_state.pose.position.x = 0
        self.model_state.pose.position.y = 0
        self.model_state.pose.position.z = 0

    def update_model_state_vel_sin(self, time):
        self.model_state.twist.linear.x = self.x_a * math.sin(self.x_f * time + self.x_p)
        self.model_state.twist.linear.y = self.y_a * math.sin(self.y_f * time + self.y_p)
        self.model_state.twist.linear.z = self.z_a * math.sin(self.z_f * time + self.z_p)


def objective(x, a, b):
    return a * sin(b*x)
'''
def datagt(msg):
    global flag, x_ser, y_ser, aa, bb
    index_of_interest = -1
    for i in range(len(msg.name)):
        if msg.name[i] == "landing_station":
            index_of_interest = i
            break
    time = rospy.get_time()
    if flag <= 10000:
        x = msg.twist[index_of_interest].linear.x
        y = msg.twist[index_of_interest].linear.y
        aa.append(time)
        bb.append(y)
        f = open("/home/toor/Desktop/test.txt", "a")

        f.write(str(x))
        f.close()

        f1 = open("/home/toor/Desktop/test1.txt", "a")
        f1.write(str(y))
        f1.close()

        flag += 1

'''

def hook():
    ls_1.update_model_state_pose_sin(0)
    model_state_pub.publish(ls_1.model_state)

t = True

if __name__ == '__main__':
    rate = 30
    rospy.init_node('moving_landing_pad')
    model_name = rospy.get_param("~model_name", "landing_station")
    seconds_before_moving = rospy.get_param("~seconds_before_moving", 0)
    rospy.sleep(int(seconds_before_moving))
    ls_1 = landing_mover(model_name, z_a=0, x_a=1.1, x_p=1.57, y_a=1.1)
    model_state_pub = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=1)
    #rospy.Subscriber("/gazebo/model_states", ModelStates, datagt)
    r = rospy.Rate(rate)
    rospy.on_shutdown(hook)
    while not rospy.is_shutdown():
        '''
        if flag >=10000 and t:
            x_ser = np.asarray(aa)
            y_ser = np.asarray(bb)
            popt, _ = curve_fit(objective, x_ser, y_ser)
            print(popt)
            a, b = popt
            pyplot.scatter(x_ser, y_ser)
            x_line = arange(min(x_ser), max(x_ser), 1)
            y_line = objective(x_line, a, b)
            # create a line plot for the mapping function
            pyplot.plot(x_line, y_line, '--', color='red')
            pyplot.show()
            t = False
            '''
        try:
            time = rospy.get_time()
            ls_1.update_model_state_vel_sin(time)
            model_state_pub.publish(ls_1.model_state)
            r.sleep()
        except rospy.ROSInterruptException:
            pass
            # rospy.logerr("ROS Interrupt Exception!")
        except rospy.ROSTimeMovedBackwardsException:
            rospy.logerr("ROS Time Backwards! Likely simulator reset")