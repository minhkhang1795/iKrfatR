import numpy as np
import math
import cv2

# LOAD the image and show it
image = cv2.imread("cube2.JPG")
# cv2.imshow("original", image)
# cv2.waitKey(0)

# RESIZING the image
# we need to keep in mind aspect ratio therefore, we calculate the ratio of the new image to the old image
r = 500.0 / image.shape[1]
dim = (500, int(image.shape[0] * r))
# perform the actual resizing of the image and show it
resized = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)
# cv2.imshow("resized", resized)
# cv2.waitKey(0)

# PROCESSING the image
# convert the image to grayscale, blur it, and find edges
gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
gray = cv2.bilateralFilter(gray, 11, 17, 17)
edged = cv2.Canny(gray, 30, 200)
# cv2.imshow("resized", edged)
# cv2.waitKey(0)

#find the contours
_, contours, _= cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = sorted(contours, key = cv2.contourArea, reverse = True)[:3]
# cv2.drawContours(resized, [cnts[0]], -1, (0, 255, 0), 3)
# cv2.imwrite("contour.jpg", resized)
screenCnt = cnts[0] #the largest countour is most likely the stack of cubes

# CROP make bounding rectangle around stack and crop out
x,y,w,h = cv2.boundingRect(screenCnt)
img = cv2.rectangle(resized,(x,y),(x+w,y+h),(0,255,0),2)
# cv2.imshow("thresh", img)
# cv2.waitKey(0)
crop_img = img[y:y+h, x:x+w]
# cv2.imshow("cropped", crop_img)
# cv2.waitKey(0)

# ANALYZE find lines in the newly cropped area
gray2 = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
edged2 = cv2.Canny(gray2,50,150,apertureSize = 3)
# cv2.imshow("cropped", edged2)
# cv2.waitKey(1000)

for i in range(10, 60):
    theta = math.pi * i / 180
    lines = cv2.HoughLinesP(edged2, 1, theta, 2, 80, 40, 5 )
    if lines[0] is not None:
        for i in range(0, len(lines)):
            line = lines[i][0]
            pt1 = (line[0],line[1])
            pt2 = (line[2],line[3])
            angle = np.arctan2(line[3] - line[1], line[2] - line[0]) * 180. / np.pi
            if angle != 90 and angle != -90 and 30 > angle > -30:
                cv2.line(crop_img, pt1, pt2, (0,0,255), 3)

cv2.imwrite("result2.jpg", crop_img)