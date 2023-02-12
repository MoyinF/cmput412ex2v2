#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import LEDPattern
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import Header, ColorRGBA
from duckietown_msgs.srv import SetFSMState, SetFSMStateResponse

class LEDServiceNode(DTROS):
  def __init__(self, node_name):
    super(LEDServiceNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
    self.veh_name = rospy.get_namespace().strip("/")

    # Services
    self.led_service = rospy.Service(f'/{self.veh_name}/led_service_node/led_service', SetFSMState, self.set_pattern)
    
    # Publishers
    self.led_pattern_pub = rospy.Publisher(f'/{self.veh_name}/led_emitter_node/led_pattern', LEDPattern, queue_size=1)

    # Color options
    self.rgba_colors = {
        'RED': [1.0, 0.0, 0.0, 1.0],
        'GREEN': [0.0, 1.0, 0.0, 1.0],
        'BLUE': [0.0, 0.0, 0.0, 1.0],
        'PURPLE': [1.0, 0.0, 1.0, 1.0],
        'YELLOW': [1.0, 1.0, 0.0, 1.0],
        'CYAN': [0.0, 1.0, 1.0, 1.0],
        'WHITE': [1.0, 1.0, 1.0, 1.0],
    }

  def set_pattern(self, request):
    desired_color = request.state

    if desired_color not in self.rgba_colors:
      return SetFSMStateResponse()

    
    pattern = LEDPattern()
    pattern.header = Header()
    pattern.header.stamp = rospy.Time.now()
    
    rgba = ColorRGBA()
    rgba.r = self.rgba_colors[desired_color][0]
    rgba.g = self.rgba_colors[desired_color][1]
    rgba.b = self.rgba_colors[desired_color][2]
    rgba.a = self.rgba_colors[desired_color][3]
    
    pattern.rgb_vals = [rgba] * 5
    self.led_pattern_pub.publish(pattern)

    return SetFSMStateResponse()

if __name__ == '__main__':
  # Create the LEDServiceNode object
  led_service_node = LEDServiceNode(node_name="led_service_node")
  # Keep it spinning to keep the node alive
  rospy.spin()
