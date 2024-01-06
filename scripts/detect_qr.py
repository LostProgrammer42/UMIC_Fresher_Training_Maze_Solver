import cv2
img = cv2.imread('/home/stavan/camera_ws/src/rover_description/scripts/frame.png')

img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
thresh = 40
img = cv2.threshold(img, thresh, 255, cv2.THRESH_BINARY)[1]
qcd = cv2.QRCodeDetector()
is_qr_there, decoded_info,b = qcd.detectAndDecode(img)
print(is_qr_there)
cv2.imshow('qr',img)
cv2.waitKey(100)