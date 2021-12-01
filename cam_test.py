import rospy
from pacmod_msgs.msg import PacmodCmd
import cv2
import torch
from cv_bridge import CvBridge, CvBridgeError
##FOr Webcam
class image_converter:
    def __init__(self):
        # self.image_pub = rospy.Publisher("image_topic_2",Image)

        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
        self.image_sub = rospy.Subscriber("/mako_1/mako_1/image_raw", PacmodCmd, self.callback)

    def callback(self,data):
        global cv_image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)

def main():
    global cv_image
    rospy.init_node('Polaris GEM 2 Ex')
    pub = rospy.Publisher("/pacmod/as_rx/turn_cmd", PacmodCmd, queue_size = 10)
    # sub = rospy.Subscriber("/mako_1/mako_1/image_raw", PacmodCmd, callback)
    #YOLO Network Here
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
    count = 0
    while not rospy.is_shutdown():
        while not pub.get_num_connections() == 1:
            pass
        ##What we need for image detection
        results = model(cv_image) ## frame == image
        temp = results.xyxy[0][:,5].tolist()
        #image from the camera
        if 0.0 in temp:
            print("PERSON DETECTED")

if __name__ == "__main__":
    main()



# import cv2
# import torch
# ##FOr Webcam
# # vid = cv2.VideoCapture(0)
# vid = cv2.VideoCapture(0,cv2.CAP_DSHOW)
#
# model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
# count = 0
# while True:
#     ##Webcam
#     ret, frame= vid.read()
#     cv2.imshow('frame', frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
#
#     ##What we need for image detection
#     results = model(frame) ## frame == image
#     temp = results.xyxy[0][:,5].tolist()
#     if 0.0 in temp:
#       print("PERSON DETECTED")
#       count += 1
#       print(count)
#
#
#
# vid.release()
# cv2.detroyAllWindows()
