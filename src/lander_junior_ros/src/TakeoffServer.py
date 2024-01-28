#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from lander_junior_ros.msg  import PositionalState, Force

class TakeoffServer():

    def __init__(self):

        # ESC Enable Service:
        self.esc_enable_service = rospy.get_param("~services/EnableESC")

        # Servo Enable Service:
        self.servo_enable_service = rospy.get_param("~services/EnableServo")

        # ESC Disable Service:
        self.esc_disable_service = rospy.get_param("~services/DisableESC")

        # Servo Disable Service:
        self.servo_disable_service = rospy.get_param("~services/DisableServo")

        # Takeoff Service:
        takeoff_service = rospy.get_param("~services/Takeoff")
        rospy.Service(takeoff_service,Trigger,self.takeoff_callback)

        # Land Service:
        land_service = rospy.get_param("~services/Land")
        rospy.Service(land_service,Trigger,self.land_callback)

        # State Estimate Subscriber:
        positional_state_topic = rospy.get_param("~topics/positional_state")
        self.positional_state_sub = rospy.Subscriber(positional_state_topic,PositionalState,self.callback_positional_state)

        # Reference Position Publisher:
        reference_positional_state_topic = rospy.get_param("~topics/reference_positional_state")
        self.reference_positional_state_pub = rospy.Publisher(reference_positional_state_topic,PositionalState,queue_size=10)

        # Feedforward Force Subscriber:
        feedforward_force_topic = rospy.get_param("~topics/feedforward_force")
        self.feedforward_force_pub = rospy.Publisher(feedforward_force_topic,Force,queue_size=10)

        # Takeoff Parameters:
        self.hover_height = rospy.get_param("/HoverHeight")
        self.hover_reference_force = rospy.get_param("HoverFeedforwardForce")

        self.PositionalStateEstimate[0] = 0
        self.PositionalStateEstimate[1] = 0
        self.PositionalStateEstimate[2] = 0
        self.PositionalStateEstimate[3] = 0
        self.PositionalStateEstimate[4] = 0
        self.PositionalStateEstimate[5] = 0

    def takeoff_callback(self,req):
        
        # Call Serice to Activate Servos
        try:
            rospy.wait_for_service(self.servo_enable_service)
            servo_service = rospy.ServiceProxy(self.servo_enable_service,Trigger)
            servo_enabled = servo_service()
            rospy.loginfo("Servo Service Called")
        except:
            rospy.logerr(f"Servo Servive Failed")

        # Sleep for 2 seconds
        rospy.sleep(2) 

        # Call Service to Activate ESCs
        try:
            rospy.wait_for_service(self.esc_enable_service)
            esc_service = rospy.ServiceProxy(self.esc_enable_service,Trigger)
            esc_enabled = esc_service()
            rospy.loginfo("ESC Service Called")
        except:
            rospy.logerr(f"ESC Servive Failed")

        if not hasattr(self,'PositionalStateEstimate'):
            rospy.logerr(f"No Positional Estimate Yet!")

        # Publish feedforward force and reference position
        feedforward_force_msg = Force()
        feedforward_force_msg.F1 = self.hover_reference_force[0]
        feedforward_force_msg.F2 = self.hover_reference_force[1]
        feedforward_force_msg.F3 = self.hover_reference_force[2]

        reference_position_msg = PositionalState()
        reference_position_msg.x = self.PositionalStateEstimate[0]
        reference_position_msg.y = self.PositionalStateEstimate[1]
        reference_position_msg.z = self.PositionalStateEstimate[2]+self.hover_height
        reference_position_msg.vx = 0
        reference_position_msg.vy = 0
        reference_position_msg.vz = 0

        self.feedforward_force_pub.publish(feedforward_force_msg)
        self.reference_positional_state_pub.publish(reference_position_msg)

        self.ground_height = self.PositionalStateEstimate[2]

        return TriggerResponse(success=servo_enabled and esc_enabled,message="Takeoff Executed")
    
    def land_callback(self,req):

        # Publish feedforward force and reference position

        reference_position_msg = PositionalState()
        reference_position_msg.x = self.PositionalStateEstimate[0]
        reference_position_msg.y = self.PositionalStateEstimate[1]
        reference_position_msg.z = self.PositionalStateEstimate[2]+self.hover_height
        reference_position_msg.vx = 0
        reference_position_msg.vy = 0
        reference_position_msg.vz = 0

        self.reference_positional_state_pub.publish(reference_position_msg)

        rospy.Duration(5).sleep() # Wait for settling

        feedforward_force_msg = Force()
        feedforward_force_msg.F1 = 0
        feedforward_force_msg.F2 = 0
        feedforward_force_msg.F3 = 0
        self.feedforward_force_pub.publish(feedforward_force_msg)

        # Call Service to Disable ESCs
        try:
            rospy.wait_for_service(self.esc_disable_service)
            esc_service = rospy.ServiceProxy(self.esc_disable_service,Trigger)
            esc_disabled = esc_service()
            rospy.loginfo("ESC Service Called")
        except:
            rospy.logerr(f"ESC Servive Failed")

        # Sleep for 2 seconds
        rospy.sleep(2) 

        # Call Serice to Disable Servos
        try:
            rospy.wait_for_service(self.servo_disable_service)
            servo_service = rospy.ServiceProxy(self.servo_disable_service,Trigger)
            servo_disabled = servo_service()
            rospy.loginfo("Servo Service Called")
        except:
            rospy.logerr(f"Servo Servive Failed")

        return TriggerResponse(success=servo_disabled and esc_disabled,message="Landing Executed")

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


if __name__ == "__main__":
    rospy.init_node('takeoff_server')
    TakeoffServer()
    rospy.spin()
