import rospy
from pacmod_msgs.msg import PacmodCmd

def stop_light(pacmod_msg):
    pacmod_msg.TURN_NONE = 1
    pub.publish(pacmod_msgs)

def right_light(pacmod_msg):
    pacmod_msg.TURN_RIGHT = 0
    pub.publish(pacmod_msgs)

def left_light(pacmod_msg):
    pacmod_msg.TURN_LEFT = 2
    pub.publish(pacmod_msgs)

def main(pacmod_msg):
    sleep_time = 3

    left_light(pacmod_msg)
    rospy.sleep(sleep_time)
    stop_light(pacmod_msg)

    right_light(pacmod_msg)
    rospy.sleep(sleep_time)
    stop_light(pacmod_msg)

    left_light(pacmod_msg)
    rospy.sleep(sleep_time)
    stop_light(pacmod_msg)

    rospy.sleep(sleep_time)

if __name__ == "__main__":
    rospy.init_node('Polaris GEM 2 Ex')
    pub = rospy.Publisher("/pacmod/as_rx/turn_cmd", PacmodCmd, queue_size = 10)
    while not rospy.is_shutdown():
        while not pub.get_num_connections() == 1:
            pass

        pacmod_msg = PacmodCmd()
        main(pacmod_msg)
