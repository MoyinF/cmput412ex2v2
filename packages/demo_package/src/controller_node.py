#!/usr/bin/env python3
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String, Header, Float32, ColorRGBA
from duckietown_msgs.msg import LEDPattern, WheelsCmdStamped
from duckietown_msgs.srv import ChangePattern
import time

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

        # Service proxies
        # rospy.wait_for_service(f'/{self.veh_name}/led_service_node/led_service')
        self.led_service = rospy.ServiceProxy(f'/{self.veh_name}/led_service_node/led_service', ChangePattern)

        # vehicle speeds
        self.x_speed = 0.3
        self.turn_speed = 0.55
        self.circular_speed = 0.4

        # wheel distances
        self.distance_left = 0
        self.distance_right = 0

        self.l = 0.05 # meters between the center of the wheel and the robot rotation
        self.r = 0.033	# radius of the wheel in meteres

        # set shutdown behaviour
        rospy.on_shutdown(self.stop)


    def run(self):
        self.state_1(5)
        self.state_2()
        self.state_1(5)
        self.state_3()
        self.state_1(5)
        self.state_4()

    def state_1(self, seconds):
        # change LEDs and wait for 5 seconds
        self.publish_leds("RED")
        time.sleep(seconds)

    def state_2(self):
        # change LEDs, move 90 degrees and forward 3 times
        self.publish_leds("GREEN")
        self.right_turn(np.pi/2)
        self.forward(1.25)
        self.left_turn(np.pi/2)
        self.forward(1.25)
        self.left_turn(np.pi/2)
        self.forward(1.25)
        

    def state_3(self):
        # change LEDs and return to initial position
        self.publish_leds("BLUE")
        self.right_turn(np.pi/2)
        self.backward(1.25)
        self.stop()

    def state_4(self):
        # change LEDs and move in circular motion
        self.publish_leds("WHITE")
        self.forward(0.3) # move a bit forward
        self.circular_motion(0.6)

    def forward(self, total_distance):
        starting_distance = (self.distance_left + self.distance_right) / 2
        current_distance = starting_distance

        msg = WheelsCmdStamped()
        msg.vel_left = self.x_speed
        msg.vel_right = self.x_speed

        while current_distance - starting_distance < total_distance:
            current_distance = (self.distance_left + self.distance_right) / 2
            self.pub_wheel_commands.publish(msg)

        self.stop()


    def backward(self, total_distance):
        starting_distance = (self.distance_left + self.distance_right) / 2
        current_distance = starting_distance

        msg = WheelsCmdStamped()
        msg.vel_left = -self.x_speed
        msg.vel_right = -self.x_speed

        rospy.loginfo("left:" + str(msg.vel_left) + " right: " + str(msg.vel_right))

        while starting_distance - current_distance < total_distance:
            current_distance = (self.distance_left + self.distance_right) / 2
            self.pub_wheel_commands.publish(msg)

        self.stop()

    def rot_dist(self, angle):
        # angle is in radians
        # r * theta is formula for arc length
        return angle * self.l

    def left_turn(self, angle):
        # total_distance is how far each wheel has to travel
        total_distance = self.rot_dist(angle)

        starting_distance_l = self.distance_left
        starting_distance_r = self.distance_right
        current_distance = 0

        msg = WheelsCmdStamped()
        msg.vel_left = -self.turn_speed
        msg.vel_right = self.turn_speed

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
        msg.vel_left = self.turn_speed
        msg.vel_right = -self.turn_speed

        while current_distance < total_distance:
            rospy.loginfo("right_turn current_distance " + str(current_distance))
            current_distance = (abs(starting_distance_l - self.distance_left) + abs(starting_distance_r - self.distance_right))/2
            self.pub_wheel_commands.publish(msg)
        self.stop()

    def circular_motion(self, rad):
        # for clockwise motion
        msg = WheelsCmdStamped()

        # rad is center of rotation to center of duckiebot,
        # so need big & small radii for each wheel
        big_distance = 2 * np.pi * (rad + self.l)
        small_distance = 2 * np.pi * (rad - self.l)

        starting_distance_l = self.distance_left
        starting_distance_r = self.distance_right

        vel_ratio = big_distance / small_distance

        # clockwise motion: left wheel is the outer wheel, so it needs to have the bigger distance
        msg.vel_left = self.circulat_speed * vel_ratio
        msg.vel_right = self.circular_speed
        
        while (self.distance_left - starting_distance_l < big_distance) or (self.distance_right - starting_distance_r < small_distance):
            self.pub_wheel_commands.publish(msg)
        self.stop()

    def stop(self):
        msg = WheelsCmdStamped()
        msg.vel_left = 0.0
        msg.vel_right = 0.0
        self.pub_wheel_commands.publish(msg)

    def publish_leds(self, color):
        try:
            self.led_service(String(color))
        except:
            rospy.loginfo("Failed to publish LEDs")

    def cb_distance_left(self, msg):
        self.distance_left = msg.data

    def cb_distance_right(self, msg):
        self.distance_right = msg.data

if __name__ == '__main__':
    # create the node
    node = ControllerNode(node_name='controller_node')
    # run node
    node.run()
    reason = "Program over. Exiting."
    rospy.signal_shutdown(reason)
