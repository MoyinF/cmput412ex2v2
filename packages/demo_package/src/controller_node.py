#!/usr/bin/env python3
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String, Header, Float32, ColorRGBA
from duckietown_msgs.msg import LEDPattern, WheelsCmdStamped
from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern

class ControllerNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ControllerNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.veh_name = rospy.get_namespace().strip("/")

        # Subscribers
        self.sub_distance_left = rospy.Subscriber(f'/{self.veh_name}/odometry_node/integrated_distance_left', Float32, self.cb_distance_left)
        self.sub_distance_right = rospy.Subscriber(f'/{self.veh_name}/odometry_node/integrated_distance_right', Float32, self.cb_distance_right)

        # Publishers
        self.pub_wheel_commands = rospy.Publisher(f'/{self.veh_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)

        # Services
        rospy.wait_for_service(f'/{self.veh_name}/led_emitter_node/set_pattern')
        self.led_service = rospy.ServiceProxy(f'/{self.veh_name}/led_emitter_node/set_pattern', ChangePattern)

        # vehicle speeds
        self.x_speed = 0.3
        self.turn_speed = 0.45
        self.slow_down = 0.15 # for slowing down turns during rotations
        self.speed_up = 0.055 # for speeding up the lagging wheel

        # self.x_speed = 0.4
        # self.turn_speed = 0.15

        # wheel distances
        self.distance_left = 0
        self.distance_right = 0

        self.l = 0.05 # meters between the center of the wheel and the robot rotation
        self.r = 0.033	# radius of the wheel in meteres



    def run(self):
        self.first_right_turn(np.pi/2)
        self.forward(1.25)
        self.left_turn(np.pi/2)
        self.forward(1.25)
        self.left_turn(np.pi/2)
        self.forward(1.25)
        self.right_turn(np.pi/2)
        self.stop()

        # self.left_turn(np.pi/2)
        # self.backward(0.3)
        return

    def complete(self):
        self.publish_leds("WHITE")


    def forward(self, total_distance):
        starting_distance = (self.distance_left + self.distance_right) / 2
        current_distance = starting_distance

        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = self.x_speed
        msg.vel_right = self.x_speed + self.speed_up # csc22906

        while current_distance - starting_distance < total_distance:
            current_distance = (self.distance_left + self.distance_right) / 2
            self.pub_wheel_commands.publish(msg)

        self.stop()


    def backward(self, total_distance):
        starting_distance = (self.distance_left + self.distance_right) / 2
        current_distance = starting_distance

        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = -self.x_speed
        msg.vel_right = -self.x_speed

        rospy.loginfo("left:" + str(msg.vel_left) + " right: " + str(msg.vel_right))

        while starting_distance - current_distance < total_distance:
            current_distance = (self.distance_left + self.distance_right) / 2
            self.pub_wheel_commands.publish(msg)

        self.stop()

    def rot_dist(self, angle):
        # angle is in radians
        # r*theta is formula for arc length
        return angle * self.l

    def left_turn(self, angle):
        # total_distance is how far each wheel has to travel
        total_distance = self.rot_dist(angle)

        starting_distance_l = self.distance_left
        starting_distance_r = self.distance_right
        current_distance = 0

        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()

        msg.vel_left = -self.turn_speed
        msg.vel_right = self.turn_speed + self.slow_down # csc22906

        while current_distance < total_distance:
            rospy.loginfo("left_turn current_distance " + str(current_distance))
            current_distance = (abs(starting_distance_l - self.distance_left) + abs(starting_distance_r - self.distance_right))/2
            self.pub_wheel_commands.publish(msg)
        self.stop()

    def right_turn(self, angle):
        total_distance = self.rot_dist(angle)

        starting_distance_l = self.distance_left
        starting_distance_r = self.distance_right
        current_distance = 0

        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()

        msg.vel_left = self.turn_speed
        msg.vel_right = -self.turn_speed

        while current_distance < total_distance:
            rospy.loginfo("right_turn current_distance " + str(current_distance))
            current_distance = (abs(starting_distance_l - self.distance_left) + abs(starting_distance_r - self.distance_right))/2
            self.pub_wheel_commands.publish(msg)
        self.stop()

    def first_right_turn(self, angle):
        total_distance = self.rot_dist(angle)

        starting_distance_l = 0
        starting_distance_r = 0
        current_distance = 0

        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()

        msg.vel_left = self.turn_speed
        msg.vel_right = -self.turn_speed + self.slow_down # csc22906
        self.pub_wheel_commands.publish(msg)

        while current_distance < total_distance:
            rospy.loginfo("first_right_turn current_distance " + str(current_distance))
            current_distance = (abs(starting_distance_l - self.distance_left) + abs(starting_distance_r - self.distance_right))/2
            self.pub_wheel_commands.publish(msg)
        self.stop()


    def stop(self):
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = 0.0
        msg.vel_right = 0.0
        self.pub_wheel_commands.publish(msg)

    def publish_leds(self, color):
        try:
            response = self.led_service(String(color))
            rospy.loginfo(response)
        except Exception as e:
            rospy.loginfo("Failure to change LED pattern:" + str(e))

    def cb_distance_left(self, msg):
        self.distance_left = msg.data

    def cb_distance_right(self, msg):
        self.distance_right = msg.data

if __name__ == '__main__':
    # create the node
    node = ControllerNode(node_name='controller_node')
    # run node
    node.run()
    # keep spinning
    # rospy.spin()
    reason = "Program over. Exiting."
    rospy.signal_shutdown(reason)
