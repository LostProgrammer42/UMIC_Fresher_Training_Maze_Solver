import cv2

img = cv2.imread('/home/stavan/camera_ws/src/rover_description/scripts/frame1.png')
l,w,a = img.shape
for i in range(l):
    for j in range(w):
        pixel = img[i][j]
        if 79<pixel[2]<240:
            img[i][j] = [255,255,255]

l_c = 8
w_c = 8
br = False
for i in range(l-l_c):
    for j in range(w-w_c):
        pixel = img[i][j]
        if pixel[0]==0 and pixel[1]==0 and pixel[2]==0:
            cv2.rectangle(img,(int(j-l_c/2),int(i-w_c/2)),(int(j+l_c/2),int(i+w_c/2)),(0,255,0),1)
            br = True
            break
    if br:
        break


cv2.imshow('cam',img)
cv2.waitKey(0)
cv2.imwrite('/home/stavan/camera_ws/src/rover_description/scripts/frame_corrected.png',img)
        