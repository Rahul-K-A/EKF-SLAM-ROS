import rospy
from nav_msgs.msg import Odometry

def odom_callback(msg):
    print(f"Pose: x:{msg.pose.pose.position.x}")


def main():
    rospy.init_node("Odom listener")
    sub = rospy.Subscriber("/odom",Odometry,odom_callback)
    rospy.spin()



if __name__=="__main__":
    main()