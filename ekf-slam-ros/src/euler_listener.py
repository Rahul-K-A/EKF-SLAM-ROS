import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print(yaw," d: ",math.degrees(yaw))


if __name__ == "__main__":
    rospy.init_node('my_quaternion_to_euler')
    sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        r.sleep()