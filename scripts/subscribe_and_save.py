#!/usr/bin/env python
import rospy
from std_msgs.msg import String  # Replace with the actual message type
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped
# ...
class TopicSubscriber:
    def __init__(self, topic_name, message_type, output_file):
        self.output_file = output_file
        rospy.Subscriber(topic_name, message_type, self.callback)
        self.file = open(output_file, 'w')

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        self.file.write(str(data) + '\n')

    def close_file(self):
        self.file.close()

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    topic_name = 'pose_topic'  # Replace with your topic
    message_type = PoseWithCovarianceStamped
    output_file = 'output.txt'

    subscriber = TopicSubscriber(topic_name, message_type, output_file)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        subscriber.close_file()

