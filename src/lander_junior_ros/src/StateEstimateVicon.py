#!/usr/bin/env python3

import rospy
import numpy as np

from lander_junior_ros.msg  import AttitudeState, PositionalState
from geometry_msgs.msg import TransformStamped

class ViconStateEstimator:
    """
    This class estimates the state of lander junior using the vicon camera system
    """

    def __init__(self):
        # Subscribers:
        #   vicon_transform
        # Publishers:
        #   attitude_state
        #   positional_state

        # Vicon Subscriber:
        vicon_transform_topic = rospy.get_param("~topics/vicon_transform")
        self.vicon_transform_sub = rospy.Subscriber("/vicon/robot_1/robot_1",TransformStamped,self.callback_vicon)

        # Attitude State Publisher
        attitude_state_topic = rospy.get_param("~topics/attitude_state")
        self.attitude_state_pub = rospy.Publisher(attitude_state_topic,AttitudeState,queue_size=10)

        # Position State Publisher
        positional_state_topic = rospy.get_param("~topics/positional_state")
        self.positional_state_pub = rospy.Publisher(positional_state_topic,PositionalState,queue_size=10)

        # Estimate Parameters:
        self.dt = rospy.get_param("/ViconEstimatorRate")


    def callback_vicon(self,vicon_transform_msg):
        self.q0 = vicon_transform_msg.transform.rotation.w
        self.q1 = vicon_transform_msg.transform.rotation.x
        self.q2 = vicon_transform_msg.transform.rotation.y
        self.q3 = vicon_transform_msg.transform.rotation.z

        self.x = vicon_transform_msg.transform.translation.x
        self.y = vicon_transform_msg.transform.translation.y
        self.z = vicon_transform_msg.transform.translation.z

    def publish_estimates(self):

        if hasattr(self,'xprev'):
            vx = (self.x-self.xprev)/self.dt
            vy = (self.y-self.yprev)/self.dt
            vz = (self.z-self.zprev)/self.dt

            w1 = -2/self.dt*(self.q3*self.q0prev + self.q2*self.q1prev - self.q1*self.q2prev -self.q0*self.q3prev)
            w2 = -2/self.dt*(-self.q2*self.q0prev + self.q3*self.q1prev + self.q0*self.q2prev -self.q1*self.q3prev)
            w3 = -2/self.dt*(self.q1*self.q0prev - self.q0*self.q1prev + self.q3*self.q2prev -self.q2*self.q3prev)
        else:
            vx = 0
            vy = 0
            vz = 0
            
            w1 = 0
            w2 = 0
            w3 = 0

        self.xprev = self.x
        self.yprev = self.y
        self.zprev = self.z

        self.q0prev = self.q0
        self.q1prev = self.q1
        self.q2prev = self.q2
        self.q3prev = self.q3

        attitude_state_msg = AttitudeState()
        attitude_state_msg.q0 = self.q0
        attitude_state_msg.q1 = self.q1
        attitude_state_msg.q2 = self.q2
        attitude_state_msg.q3 = self.q3
        attitude_state_msg.w1 = w1
        attitude_state_msg.w2 = w2
        attitude_state_msg.w3 = w3

        positional_state_msg = PositionalState()
        positional_state_msg.x = self.x
        positional_state_msg.y = self.y
        positional_state_msg.z = self.z
        positional_state_msg.vx = vx
        positional_state_msg.vy = vy
        positional_state_msg.vz = vz

        self.attitude_state_pub.publish(attitude_state_msg)
        self.positional_state_pub.publish(positional_state_msg)



if __name__ == "__main__":
    rospy.init_node("vicon_state_estimator_node")
    Estimator = ViconStateEstimator()

    rate = rospy.Rate(rospy.get_param("/ViconEstimatorRate"))

    while not rospy.is_shutdown():
        Estimator.publish_estimates()
        rate.sleep()
