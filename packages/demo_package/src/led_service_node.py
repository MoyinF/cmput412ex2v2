#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.srv import ChangePattern
from std_msgs.msg import String

class LEDServiceNode(DTROS):
  def __init__(self, node_name):
    super(LEDServiceNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
    self.veh_name = rospy.get_namespace().strip("/")
    
    # Services proxies
    # rospy.wait_for_service(f'/{self.veh_name}/led_emitter_node/set_pattern')
    self.set_pattern = rospy.ServiceProxy(f'/{self.veh_name}/led_emitter_node/set_pattern', ChangePattern)
    
    # Services
    self.led_service = rospy.Service(f'/{self.veh_name}/led_service_node/led_service', ChangePattern, self.set_pattern_cb)

  def set_pattern_cb(self, msg):
    return self.set_pattern(String(msg.pattern_name.data))

if __name__ == '__main__':
  # Create the LEDServiceNode object
  led_service_node = LEDServiceNode(node_name="led_service_node")
  # Keep it spinning to keep the node alive
  rospy.spin()
