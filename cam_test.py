import cv2
import torch
##FOr Webcam
# vid = cv2.VideoCapture(0)
vid = cv2.VideoCapture(0,cv2.CAP_DSHOW)

model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
count = 0
while True:
    ##Webcam
    ret, frame= vid.read()
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    ##What we need for image detection
    results = model(frame) ## frame == image
    temp = results.xyxy[0][:,5].tolist()
    if 0.0 in temp:
      print("PERSON DETECTED")
      count += 1
      print(count)



vid.release()
cv2.detroyAllWindows()
