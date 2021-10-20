import rospy
from pacmod_msgs.msg import PacmodCmd
import torch


def brake(pub):
    pacmod_msg = PacmodCmd()
    pacmod_msg.enable = True
    pacmod_msg.clear = False
    pacmod_msg.ignore = False
    pacmod_msg.f64_cmd = 1.0
    pub.publish(pacmod_msgs)

def unbrake(pub):
    pacmod_msg = PacmodCmd()
    pacmod_msg.enable = True
    pacmod_msg.clear = False
    pacmod_msg.ignore = False
    pacmod_msg.f64_cmd = 0.0
    pub.publish(pacmod_msgs)

def callback(msg):
    global cam_image
    cam_image = msg


def main():
    global cam_image
    rospy.init_node('Polaris GEM 2 Ex')
    pub = rospy.Publisher("/pacmod/as_rx/turn_cmd", PacmodCmd, queue_size = 10)
    sub = rospy.Subscriber("/mako_1/mako_1/image_raw", PacmodCmd, callback)
    #YOLO Network Here
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
    count = 0
    while not rospy.is_shutdown():
        while not pub.get_num_connections() == 1:
            pass

        ##What we need for image detection
        results = model(cam_image) ## frame == image
        temp = results.xyxy[0][:,5].tolist()
        #image from the camera
        if 0.0 in temp:
            print("PERSON DETECTED")
            brake(pub)
            count += 1
            print(count)
            # while (0.0 in temp):
            #     print("PERSON DETECTED")
            #     brake(pub)
            #     count += 1
            #     print(count)
        else:
            unbrake(pub)
            print("NO Brake")

if __name__ == "__main__":
    main()
