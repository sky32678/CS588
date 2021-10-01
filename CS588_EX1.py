import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def stop_ligt():
    return

def right_light():
    return

def left_light():
    return

if __name__ == "__main__":
    rospy.init_node('check_odometry')
    # sub = rospy.Subscriber("/odometry/filtered", Odometry, callback)
    pub = rospy.Publisher("/cmd_vel",Twist,queue_size =10)
    while not rospy.is_shutdown():
        while not pub.get_num_connections() == 1:
            pass

        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = control_law

        pub.publish(twist_msg)
        rospy.sleep(0.001)
