import rospy
from pacmod_msgs.msg import PacmodCmd

def stop_light(pub):
    pacmod_msg = PacmodCmd()
    # pacmod_msg.TURN_NONE = 1
    pacmod_msg.ui16_cmd = 1
    pub.publish(pacmod_msg)

def right_light(pub):
    pacmod_msg = PacmodCmd()
    # pacmod_msg.TURN_RIGHT = 0
    pacmod_msg.ui16_cmd = 0
    pub.publish(pacmod_msg)

def left_light(pub):
    pacmod_msg = PacmodCmd()
    # pacmod_msg.TURN_LEFT = 2
    pacmod_msg.ui16_cmd = 2
    pub.publish(pacmod_msg)

def main(pub):
    sleep_time = 2

    left_light(pub)
    rospy.sleep(sleep_time)
    stop_light(pub)

    right_light(pub)
    rospy.sleep(sleep_time)
    stop_light(pub)

    left_light(pub)
    rospy.sleep(sleep_time)
    stop_light(pub)

    rospy.sleep(sleep_time)

if __name__ == "__main__":
    rospy.init_node('Polaris GEM 2 Ex')
    pub = rospy.Publisher("/pacmod/as_rx/turn_cmd", PacmodCmd, queue_size = 10)
    while not rospy.is_shutdown():
        while not pub.get_num_connections() == 1:
            pass

        main(pub)
    stop_light(pub)
