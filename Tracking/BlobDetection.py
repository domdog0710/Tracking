import cv2
import numpy as np
import socket
import os

# 選擇第二隻攝影機
cap = cv2.VideoCapture(0)
cap.set(3, 1280)
cap.set(4, 720)

f = open("d:/Research/Infrared Camera/Hello/Settings.txt", "r")

ip = f.readline().split(":",1)[1]
port = int(f.readline().split(":",1)[1])

lowerwhite = int(f.readline().split(":",1)[1])
upperwhite = int(f.readline().split(":",1)[1])

minRepeatability = int(f.readline().split(":",1)[1])
minDistBetweenBlobs = float(f.readline().split(":",1)[1])

thresholdStep = int(f.readline().split(":",1)[1])
minThreshold = int(f.readline().split(":",1)[1])
maxThreshold = int(f.readline().split(":",1)[1])

filterByColor = bool(f.readline().split(":",1)[1])
blobColor = int(f.readline().split(":",1)[1])

filterByInertia = bool(f.readline().split(":",1)[1])
minInertiaRatio = float(f.readline().split(":",1)[1])
maxInertiaRatio = float(f.readline().split(":",1)[1])

filterByCircularity = bool(f.readline().split(":",1)[1])
minCircularity = float(f.readline().split(":",1)[1])
maxCircularity = float(f.readline().split(":",1)[1])

filterByConvexity = bool(f.readline().split(":",1)[1])
minConvexity = float(f.readline().split(":",1)[1])
maxConvexity = float(f.readline().split(":",1)[1])

filterByArea = bool(f.readline().split(":",1)[1])
minArea = float(f.readline().split(":",1)[1])
maxArea = float(f.readline().split(":",1)[1])

blurtime = int(f.readline().split(":",1)[1])

base = None

connected = False

while(True):  
  if(not connected):
      try:
          unity = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #TCP宣告
          unity.connect(("127.0.0.1",port))
          connected = True
      except:
          pass

  if cv2.waitKey(1) & 0xFF == ord('s'):
      f = open("d:/Research/Infrared Camera/Hello/Settings.txt", "r")
      ip = f.readline().split(":",1)[1]
      port = int(f.readline().split(":",1)[1])
      lowerwhite = int(f.readline().split(":",1)[1])
      upperwhite = int(f.readline().split(":",1)[1])
      minRepeatability = int(f.readline().split(":",1)[1])
      minDistBetweenBlobs = float(f.readline().split(":",1)[1])
      thresholdStep = int(f.readline().split(":",1)[1])
      minThreshold = int(f.readline().split(":",1)[1])
      maxThreshold = int(f.readline().split(":",1)[1])
      filterByColor = bool(f.readline().split(":",1)[1])
      blobColor = int(f.readline().split(":",1)[1])
      filterByInertia = bool(f.readline().split(":",1)[1])
      minInertiaRatio = float(f.readline().split(":",1)[1])
      maxInertiaRatio = float(f.readline().split(":",1)[1])
      filterByCircularity = bool(f.readline().split(":",1)[1])
      minCircularity = float(f.readline().split(":",1)[1])
      maxCircularity = float(f.readline().split(":",1)[1])
      filterByConvexity = bool(f.readline().split(":",1)[1])
      minConvexity = float(f.readline().split(":",1)[1])
      maxConvexity = float(f.readline().split(":",1)[1])
      filterByArea = bool(f.readline().split(":",1)[1])
      minArea = float(f.readline().split(":",1)[1])
      maxArea = float(f.readline().split(":",1)[1])
      blurtime = int(f.readline().split(":",1)[1])

  # 從攝影機擷取一張影像
  ret, ir  = cap.read()

  lower_white = np.array([lowerwhite, lowerwhite, lowerwhite])
  upper_white = np.array([upperwhite, upperwhite, upperwhite])

  mask = cv2.inRange(ir, lower_white, upper_white)

  mask = cv2.convertScaleAbs(mask)

  if cv2.waitKey(1) & 0xFF == ord('c'):
      base = mask

  if base is not None:
      # cv2.imshow('base', base)
      # cv2.imshow('mask', mask)
      diff_frame = base -  mask
      diff_frame -= diff_frame.min()
      disp_frame = np.uint8(255.0 * diff_frame / float(diff_frame.max()))
      mask = disp_frame
  else:
      base = mask
      
  for i in range(1, blurtime):
      mask = cv2.medianBlur(mask,5)#濾波

  # Setup SimpleBlobDetector parameters.
  blob_params = cv2.SimpleBlobDetector_Params()

  blob_params.minRepeatability = minRepeatability
  blob_params.minDistBetweenBlobs = minDistBetweenBlobs

  # Change thresholds
  blob_params.thresholdStep = thresholdStep
  blob_params.minThreshold = minThreshold
  blob_params.maxThreshold = maxThreshold

  # Filter by Color.
  blob_params.filterByColor = filterByColor
  blob_params.blobColor = blobColor

  # Filter by Inertia
  blob_params.filterByInertia = filterByInertia
  blob_params.minInertiaRatio = minInertiaRatio
  blob_params.maxInertiaRatio = maxInertiaRatio

  # Filter by Circularity
  blob_params.filterByCircularity = filterByCircularity
  blob_params.minCircularity = minCircularity
  blob_params.maxCircularity = maxCircularity

  # Filter by Convexity
  blob_params.filterByConvexity = filterByConvexity
  blob_params.minConvexity = minConvexity
  blob_params.maxConvexity = maxConvexity

  # Filter by Area.
  blob_params.filterByArea = filterByArea
  blob_params.minArea = minArea
  blob_params.maxArea = maxArea

  detector = cv2.SimpleBlobDetector_create(blob_params)

  keypoints = detector.detect(mask)

  im_with_keypoints = cv2.drawKeypoints(mask, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

  # image_info = im_with_keypoints.shape
  # print(image_infoq)
  cv2.imshow('ir', ir)
  cv2.imshow('detected circles',im_with_keypoints)
  
  i = 1
  
  if len(keypoints) == 0:
      my_str_as_bytes = str.encode(str(len(keypoints)) + "|" + str(0) + "|" + str(0) + "|" + str(0))

  for kp in keypoints:
      my_str_as_bytes = str.encode(str(len(keypoints)) + "|" + str(i) + "|" + str(kp.pt[0]) + "|" + str(kp.pt[1]))     
      i = i + 1
      
  if(connected):
        # try:
    unity.send(my_str_as_bytes)
          # print(my_str_as_bytes)
        # except:
          # connected = False
          # pass


  
  # 若按下 q 鍵則離開迴圈
  if cv2.waitKey(1) & 0xFF == ord('q'):
    break

# 釋放攝影機
cap.release()

# 關閉所有 OpenCV 視窗
cv2.destroyAllWindows()