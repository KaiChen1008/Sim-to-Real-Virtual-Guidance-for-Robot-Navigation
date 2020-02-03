import rospy
import roslib
from geometry_msgs.msg import Twist
from klein import Klein
import json
import sys
import os

class husListener():
    app = Klein()

    def __init__(self):
        self.count = 0
        rospy.init_node('hushus')
        rospy.on_shutdown(self.shutdownROS)
        self.pub = rospy.Publisher('husky_velocity_controller/cmd_vel', Twist, queue_size=1)

    @app.route('/', methods=['POST'])
    def handle_post(self, request):
        self.count = self.count + 1
        print('receive post', self.count)
        content = json.loads(request.content.read())
        t = self.create_twist(content)
        self.pub.publish(t)
        
    def create_twist(self, j):
        t = Twist()
        t.linear.x = j['linear_x']
        t.linear.y = j['linear_y']
        t.linear.z = j['linear_z']

        t.angular.x = j['angular_x']
        t.angular.y = j['angular_y']
        t.angular.z = j['angular_z']

        return t
    def shutdownROS(self):
        print('ROS shutdown')

if __name__ == '__main__':
    try:
        os.system('rosnode kill /joy_teleop/teleop_twist_joy')
        os.system('rosnode kill /joy_teleop/cmd_vel')
    except:
        print('can not kill joy')

    try:
        print('starting husky listener ....')
        server = husListener()
        server.app.run('192.168.0.2', 3002)

    except rospy.ROSInterruptException:
        print('ROS error')
        pass
    
