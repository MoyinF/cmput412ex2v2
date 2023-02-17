#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, Header, ColorRGBA
from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern, ChangePatternResponse
from duckietown_msgs.msg import LEDPattern

class LEDServiceNode(DTROS):
  def __init__(self, node_name):
    super(LEDServiceNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
    self.veh_name = rospy.get_namespace().strip("/")

    # Service Proxy
    # rospy.wait_for_service(f'/{self.veh_name}/led_emitter_node/set_custom_pattern')
    # self.set_pattern = rospy.ServiceProxy(f'/{self.veh_name}/led_emitter_node/set_custom_pattern', SetCustomLEDPattern)
    rospy.wait_for_service(f'/{self.veh_name}/led_emitter_node/set_pattern')
    self.set_pattern = rospy.ServiceProxy(f'/{self.veh_name}/led_emitter_node/set_pattern', ChangePattern)

    # Service
    self.led_service = rospy.Service(f'/{self.veh_name}/led_service_node/led_service', ChangePattern, self.set_pattern_cb)


  def set_pattern_cb(self, msg):

    # pattern = LEDPattern()

    # pattern.color_list = [msg.pattern_name.data] * 5
    # pattern.color_mask = [1, 1, 1, 1, 1]
    # pattern.frequency = 0.0
    # pattern.frequency_mask = [0, 0, 0, 0, 0]

    # self.set_pattern(pattern)
    self.set_pattern(String(msg.pattern_name.data))

    return ChangePatternResponse()

if __name__ == '__main__':
  # Create the LEDServiceNode object
  led_service_node = LEDServiceNode(node_name="led_service_node")
  # Keep it spinning to keep the node alive
  rospy.spin()
