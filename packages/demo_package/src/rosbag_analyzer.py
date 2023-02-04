import rosbag
bag = rosbag.Bag('test.bag')
for topic, msg, t in bag.read_messages(topics=['timestamp', 'x', 'y']):
    print msg
bag.close()

