#!/usr/bin/env python3

import rospy
import numpy as np

from lander_junior_ros.msg import AttitudeState, Force, ActuatorCommands
from utils import clip, norm, normalize, TL2B

class AttitudeControl:
    """
    This class performs attitude control for lander junior
    """

    def __init__(self):
        # Subscribers:
        #   StateEstimate
        #   Thrust
        #   ReferenceVector
        # Publishers:
        #   ActuatorCommands

        # State Estimate Subscriber:
        attitude_state_topic = rospy.get_param("~topics/attitude_state")
        self.attitude_state_sub = rospy.Subscriber(attitude_state_topic,AttitudeState,self.callback_attitude_state)

        # Thrust Subscriber:
        reference_force_topic = rospy.get_param("~topics/lander_reference_force")
        self.reference_force_sub = rospy.Subscriber(reference_force_topic,Force,self.callback_reference_force)

        # Actuator Commands Publisher:
        acuator_commands_topic = rospy.get_param("~topics/actuator_commands")
        self.actuator_commands_pub = rospy.Publisher(acuator_commands_topic,ActuatorCommands,queue_size=10)

        # Attitude Control Parameters:
        self.Kp = rospy.get_param("/AttitudeProportionalGain")
        self.Kd = rospy.get_param("/AttitudeDerivativeGain")
        self.Kw3 = rospy.get_param("/AttitudeYawDerivativeGain")
        self.L = rospy.get_param("/GimbalDistance")
        self.MaxGimbalAngle = rospy.get_param("/MaxGimbalAngle")
        self.PropellerThrustCoefficient = rospy.get_param("/PropellerThrustCoefficient")
        self.PropellerMomentCoefficient = rospy.get_param("/PropellerMomentCoefficient")
        self.MaxThrustDifferential = rospy.get_param("/MaxThrustDifferential")

        self.q = np.array([1,0,0,0])
        self.F = np.array([0,0,0])
        self.w= np.array([0,0,0])

    def callback_attitude_state(self,attitude_state_msg):
        self.q[0] = attitude_state_msg.q0
        self.q[1] = attitude_state_msg.q1
        self.q[2] = attitude_state_msg.q2
        self.q[3] = attitude_state_msg.q3
        self.w[0] = attitude_state_msg.w1
        self.w[1] = attitude_state_msg.w2
        self.w[2] = attitude_state_msg.w3

    def callback_reference_force(self,reference_force_msg):
        self.F[0] = reference_force_msg.F1
        self.F[1] = reference_force_msg.F2
        self.F[2] = reference_force_msg.F3

    def Controller(self):
        Fhat = np.matmul(TL2B(self.q),np.transpose(normalize(self.F)))

        Mhat = np.transpose(normalize(np.cross(np.array([0,0,1]),Fhat)))
        ang = np.arccos(np.dot(np.array([0,0,1]),Fhat))

        self.Moment = np.subtract(self.Kp*ang*Mhat , np.array([[self.Kd*self.w[0]],[self.Kd*self.w[1]],[self.Kw3*self.w[2]]]))

    def Actuators(self):
        Thrust = norm(self.F)
        
        if Thrust == 0:
            self.T1 = 0
            self.T2 = 0
            self.alpha = 0
            self.beta = 0
        else:
            F1 = -self.Moment[1][0]/(Thrust*self.L)
            F2 = self.Moment[0][0]/(Thrust*self.L)
            ang = np.linalg.norm(np.array([F1,F2]))
            F1n = F1*np.maximum(ang,np.sin(self.MaxGimbalAngle))/np.maximum(ang,10**-6)
            F2n = F2*np.maximum(ang,np.sin(self.MaxGimbalAngle))/np.maximum(ang,10**-6)
            self.beta = np.arcsin(F1n)
            self.alpha = np.arcsin(-F2n/np.sqrt(1-F1n**2))   
            Tdiff = clip(self.PropellerThrustCoefficient/self.PropellerMomentCoefficient*self.Moment[2][0],-self.MaxThrustDifferential,self.MaxThrustDifferential)
            self.T1 = (Thrust + Tdiff)/2
            self.T2 = (Thrust - Tdiff)/2

    def publish_actuator_commands(self):

        actuator_commands_msg = ActuatorCommands()

        actuator_commands_msg.alpha = self.alpha
        actuator_commands_msg.beta = self.beta
        actuator_commands_msg.T1 = self.T1
        actuator_commands_msg.T2 = self.T2

        self.actuator_commands_pub.publish(actuator_commands_msg)

if __name__ == "__main__":
    rospy.init_node("AttitudeController")

    AC = AttitudeControl()

    # Run at Controller Rate:
    rate = rospy.Rate(rospy.get_param("/AttitudeControlRate"))

    while not rospy.is_shutdown():
        AC.Controller()
        AC.Actuators()
        AC.publish_actuator_commands()