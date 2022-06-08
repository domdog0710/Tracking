import cv2
import numpy as np 

# 選擇第二隻攝影機
cap = cv2.VideoCapture(0)

while(True):
  # 從攝影機擷取一張影像
  ret, ir = cap.read()

  hsv = cv2.cvtColor(ir, cv2.COLOR_BGR2HSV)

  # define range of white color in HSV
  lower_white = np.array([0,0,0])
  upper_white = np.array([200,200,200])

  mask = cv2.inRange(hsv, lower_white, upper_white)

  #convert mask to 3-channel image to perform subtract
  mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

  # 顯示圖片
  cv2.imshow('ir', ir)
  cv2.imshow("bw",mask)

  # 若按下 q 鍵則離開迴圈
  if cv2.waitKey(1) & 0xFF == ord('q'):
    break

# 釋放攝影機
cap.release()

# 關閉所有 OpenCV 視窗
cv2.destroyAllWindows()