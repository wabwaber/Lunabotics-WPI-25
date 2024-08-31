# Code from https://www.youtube.com/watch?v=3h7wgR5fYik


import cv2
import os

cap = cv2.VideoCapture(2)

num = 0

while cap.isOpened():

    succes, img = cap.read()

    k = cv2.waitKey(5)

    if k == 27:
        break
    elif k == ord('s'): # wait for 's' key to save and exit
        try:
            cv2.imwrite('./images/img' + str(num) + '.png', img)
            print("image saved!")
            num += 1
        except Exception as e:
            print("Error: ", e)

    cv2.imshow('Img',img)

# Release and destroy all windows before termination
cap.release()

cv2.destroyAllWindows()