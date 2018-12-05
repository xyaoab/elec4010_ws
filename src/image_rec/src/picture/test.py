import numpy as np
import cv2 
face_cascade = cv2.CascadeClassifier('/home/xinjie/elec4010_ws/src/image_rec/src/haarcascade_frontalface_default.xml')
print(face_cascade)

img = cv2.imread('/home/xinjie/Downloads/2018f_final_project/picture/pic002.jpg')

img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

faces = face_cascade.detectMultiScale(img,scaleFactor=1.1,
	minNeighbors=3,minSize=(30,30))
print ("Found {0} faces!".format(len(faces)))
for (x,y,w,h) in faces:
    cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)

cv2.imshow('img',img)
cv2.waitKey(0)
cv2.destroyAllWindows()
