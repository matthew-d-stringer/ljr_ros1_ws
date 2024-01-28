#!/usr/bin/env python3

import rospy
import numpy as np

from lander_junior_ros.msg  import PositionalState, Force
from utils import normalize

class PositionalControl:
    """
    This class performs positional control for lander junior
    """

    def __init__(self):
        # Subscribers:
        #   positional_state
        #   reference_state
        #   feedforward_force
        # Publishers:
        #   reference_force

        # State Estimate Subscriber:
        positional_state_topic = rospy.get_param("~topics/positional_state")
        self.positional_state_sub = rospy.Subscriber(positional_state_topic,PositionalState,self.callback_positional_state)

        # Reference Position Subscriber:
        reference_positional_state_topic = rospy.get_param("~topics/reference_positional_state")
        self.reference_positional_state_sub = rospy.Subscriber(reference_positional_state_topic,PositionalState,self.callback_reference_positional_state)

        # Feedforward Force Subscriber:
        feedforward_force_topic = rospy.get_param("~topics/feedforward_force")
        self.feedforward_force_sub = rospy.Subscriber(feedforward_force_topic,Force,self.callback_feedforward_force)

        # Reference Force Publisher:
        reference_force_topic = rospy.get_param("~topics/lander_reference_force")
        self.reference_force_pub = rospy.Publisher(reference_force_topic,Force,queue_size=10)

        # Positional Control Parameters:
        self.FeedbackGainMatrix = np.reshape(np.matrix(rospy.get_param("/FeedbackGainMatrix")),(3,6))
        self.L = rospy.get_param("/GimbalDistance")

        # Initialize Control Variables
        self.MaxPointingAngle = rospy.get_param("/MaxPointingAngle")
        self.MaxThrust = rospy.get_param("/MaxThrust")
        self.MinThrust = rospy.get_param("/MinThrust")

        # Initialize Vectors
        self.PositionalStateEstimate = np.zeros((6,1))
        self.ReferencePositionalState = np.zeros((6,1))
        self.FeedforwardForce = np.zeros((3,1))

    def callback_positional_state(self,positional_state_msg):
        """
        Reads State Estimate Subscriber and Stores in State Estimate Array
        """
        self.PositionalStateEstimate[0] = positional_state_msg.x
        self.PositionalStateEstimate[1] = positional_state_msg.y
        self.PositionalStateEstimate[2] = positional_state_msg.z
        self.PositionalStateEstimate[3] = positional_state_msg.vx
        self.PositionalStateEstimate[4] = positional_state_msg.vy
        self.PositionalStateEstimate[5] = positional_state_msg.vz

    def callback_reference_positional_state(self,reference_positional_state_msg):
        """
        Reads Reference State Subscriber and Stores in Reference State Array
        """
        self.ReferencePositionalState[0] = reference_positional_state_msg.x
        self.ReferencePositionalState[1] = reference_positional_state_msg.y
        self.ReferencePositionalState[2] = reference_positional_state_msg.z
        self.ReferencePositionalState[3] = reference_positional_state_msg.vx
        self.ReferencePositionalState[4] = reference_positional_state_msg.vy
        self.ReferencePositionalState[5] = reference_positional_state_msg.vz

    def callback_feedforward_force(self,feedforward_force_msg):
        """
        Reads Reference Force Subscriber and Stores in Reference Force Array
        """
        self.FeedforwardForce[0] = feedforward_force_msg.Fx
        self.FeedforwardForce[1] = feedforward_force_msg.Fy
        self.FeedforwardForce[2] = feedforward_force_msg.Fz

    def publish_reference_force(self):
        """
        Publishes Thrust Message
        """
        reference_force_msg = Force()
        reference_force_msg.F1 = self.Force[0]
        reference_force_msg.F2 = self.Force[1]
        reference_force_msg.F3 = self.Force[2]
        self.reference_force_pub.publish(reference_force_msg)
        

    def LQR(self):
        """
        Determines Unsaturated LQR Control Input Force
        """
        self.Force = np.ravel(np.add(np.matmul(self.FeedbackGainMatrix,np.subtract(self.PositionalStateEstimate,self.ReferencePositionalState)),self.FeedforwardForce))

    def Saturate(self):
        """
        Saturates LQR Control Input Force
        """
        Fz = np.maximum(self.MinThrust,self.Force[2])
        ang = np.linalg.norm(self.Force[0:2])/np.maximum(10**-6,Fz)
        Fx = self.Force[0]*np.maximum(-np.tan(self.MaxPointingAngle),np.minimum(np.tan(self.MaxPointingAngle),ang))/np.maximum(10**-6,ang)
        Fy = self.Force[1]*np.maximum(-np.tan(self.MaxPointingAngle),np.minimum(np.tan(self.MaxPointingAngle),ang))/np.maximum(10**-6,ang)
        self.Force[0] = Fx
        self.Force[1] = Fy
        self.Force[2] = Fz

if __name__ == "__main__":
    rospy.init_node("PositionalController")
    
    PC = PositionalControl()
    
    # Run at Controller Rate:
    rate = rospy.Rate(rospy.get_param("/PositionalControlRate"))
    while not rospy.is_shutdown():
        PC.LQR()
        PC.Saturate()
        PC.publish_reference_force()




