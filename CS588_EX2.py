import rospy
from pacmod_msgs.msg import PacmodCmd


def brake():
    pacmod_msg = PacmodCmd()
    pacmod_msg.f64_cmd = 1.0
    pub.publish(pacmod_msgs)


def main():
    rospy.init_node('Polaris GEM 2 Ex')
    pub = rospy.Publisher("/pacmod/as_rx/turn_cmd", PacmodCmd, queue_size = 10)

    #YOLO Network Here
    while not rospy.is_shutdown():
        while not pub.get_num_connections() == 1:
            pass

        #image from the camera
        if image == 'Person':
            while image == 'Person':
                brake()



if __name__ == "__main__":
    main()
