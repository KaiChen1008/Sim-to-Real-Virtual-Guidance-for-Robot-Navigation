import rospy
import roslib
from geometry_msgs.msg import Pose
from klein import Klein
import json

class poseListener():
    app = Klein()

    def __init__(self):
        rospy.init_node('pose_listener')
        self.pub = rospy.Publisher('slam_pose', Pose, queue_size=1)

    @app.route('/', methods=['POST'])
    def handle_post(self, request):
        print('receive post')
        content = json.loads(str(request.content.read(), encoding='utf-8'))
        t = self.create_pose(content)
        self.pub.publish(t)
        
    def create_pose(self, j):
        t = Pose()
        t.position.x = j['position_x']
        t.position.y = j['position_y']
        t.position.z = j['position_z']

        t.orientation.x = j['orientation_x']
        t.orientation.y = j['orientation_y']
        t.orientation.z = j['orientation_z']
        t.orientation.w = j['orientation_w']
        return t

if __name__ == '__main__':
    try:
        print('starting pose listener ....')
        server = poseListener()
        server.app.run('localhost', 8888)

    except rospy.ROSInterruptException:
        print('ROS error')
        pass