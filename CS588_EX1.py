import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from pacmod_msgs.msg import PacmodCmd

def stop_light():
    pacmod_msg = PacmodCmd()
    pacmod_msg.TURN_NONE = 1
    pub.publish(pacmod_msgs)
    return

def right_light():
    pacmod_msg = PacmodCmd()
    pacmod_msg.TURN_RIGHT = 0
    pub.publish(pacmod_msgs)
    return

def left_light():
    pacmod_msg = PacmodCmd()
    pacmod_msg.TURN_LEFT = 2
    pub.publish(pacmod_msgs)
    return

if __name__ == "__main__":
    rospy.init_node('check_odometry')
    pub = rospy.Publisher("/pacmod/as_rx/turn_cmd", PacmodCmd, queue_size = 10)
    while not rospy.is_shutdown():
        while not pub.get_num_connections() == 1:
            pass

        left_light()
        rospy.sleep(3)
        stop_light()

        right_light()
        rospy.sleep(3)
        stop_light()

        left_light()
        rospy.sleep(3)
        stop_light()

        rospy.sleep(3)
