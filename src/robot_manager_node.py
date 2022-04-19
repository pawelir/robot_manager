#! /usr/bin/env python3

from geometry_msgs.msg import Twist
from math import pi
from std_msgs.msg import Float64, Int32MultiArray
import rospy

class RobotManagerNode():

    def __init__(self):
        self._load_params()
        self.pub_cmd_vel_fl = rospy.Publisher('/my_robot/left_front_wheel_velocity_controller/command',
                                              Float64,
                                              queue_size=1)
        self.pub_cmd_vel_rl = rospy.Publisher('/my_robot/left_rear_wheel_velocity_controller/command',
                                              Float64,
                                              queue_size=1)
        self.pub_cmd_vel_fr = rospy.Publisher('/my_robot/right_front_wheel_velocity_controller/command',
                                              Float64,
                                              queue_size=1)
        self.pub_cmd_vel_rr = rospy.Publisher('/my_robot/right_rear_wheel_velocity_controller/command',
                                              Float64,
                                              queue_size=1)
        self.pub_cmd_vel_epos = rospy.Publisher('/my_robot/epos_cmd_vel',
                                            Int32MultiArray,
                                            queue_size=1)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self._update_robot_velocity)
        self.epos_velocity_msg = Int32MultiArray()


    def _load_params(self) -> None:
        self.track_of_wheels = rospy.get_param("robot_manager/track_of_wheels")
        self.wheel_radius = rospy.get_param("robot_manager/wheel_radius")

    def _update_robot_velocity(self, msg: Twist) -> None:
        self.calculate_wheels_veocity(msg.linear.x, msg.angular.z)
        self.publish_controller_commands()

    def calculate_wheels_veocity(self, V: float, w: float) -> None:
        self.wheel_speed_left = (2*V - w*self.track_of_wheels) / (2*self.wheel_radius)                                    
        self.wheel_speed_right = (2*V + w*self.track_of_wheels) / (2*self.wheel_radius)
        self.epos_velocity_msg.data = [int(self.wheel_speed_left/(2*pi)*60),
                                       int(self.wheel_speed_right/(2*pi)*60)] #EPOS controller accept rpm

    def publish_controller_commands(self) -> None:
        self.pub_cmd_vel_fl.publish(self.wheel_speed_left)
        self.pub_cmd_vel_rl.publish(self.wheel_speed_left)
        self.pub_cmd_vel_fr.publish(self.wheel_speed_right)
        self.pub_cmd_vel_rr.publish(self.wheel_speed_right)
        self.pub_cmd_vel_epos.publish(self.epos_velocity_msg) 

if __name__=="__main__":
    rospy.init_node('robot_manager')
    try:
        robot_manager_node = RobotManagerNode()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()