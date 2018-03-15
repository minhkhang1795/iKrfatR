import numpy as np
import math
import cv2

# LOAD the image and show it
import imutils

image = cv2.imread("test2.jpg")
# cv2.imshow("original", image)
# cv2.waitKey(0)

# RESIZING the image
# we need to keep in mind aspect ratio therefore, we calculate the ratio of the new image to the old image

r = 500.0 / image.shape[1]
dim = (500, int(image.shape[0] * r))
# perform the actual resizing of the image and show it
resized = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)
# resized = image
# cv2.imshow("resized", resized)
# cv2.waitKey(0)
# rotated = imutils.rotate_bound(resized, 90)
rotated = resized
# cv2.imshow("Rotated (Problematic)", rotated)
# cv2.waitKey(0)

# resized = image
# cv2.imshow("resized", resized)
# cv2.waitKey(0)

# PROCESSING the image
# convert the image to grayscale, blur it, and find edges
gray = cv2.cvtColor(rotated, cv2.COLOR_BGR2GRAY)
gray = cv2.GaussianBlur(gray, (5, 5), 5)
# gray = cv2.bilateralFilter(gray, 11, 17, 17)

edged = cv2.Canny(gray, 100, 100)
# ret3, thresh = cv2.threshold(edged, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
# cv2.imshow("resized", edged)
# cv2.waitKey(0)

# find the contours
_, contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = sorted(contours, key=cv2.contourArea, reverse=True)[:]
extBots = []
extLefts = []
extRights =[]
extTops = []
for cnt in cnts:
    cv2.drawContours(resized, [cnt], -1, (0, 255, 0), 3)
    extLeft = tuple(cnt[cnt[:, :, 0].argmin()][0])[0]
    extRight = tuple(cnt[cnt[:, :, 0].argmax()][0])[0]
    extTop = tuple(cnt[cnt[:, :, 1].argmin()][0])[1]
    extBot = tuple(cnt[cnt[:, :, 1].argmax()][0])[1]
    extBots.append(extBot)
    extLefts.append(extLeft)
    extRights.append(extRight)
    extTops.append(extTop)

maxRight = max(extRights)
maxBot = max(extBots)
minLeft = min(extLefts)
minTop = min(extTops)

cv2.line(resized, (maxRight, maxBot), (maxRight, minTop), (0,0,255), 3)
cv2.line(resized, (minLeft, maxBot), (minLeft, minTop), (0,0,255), 3)
cv2.line(resized, (maxRight, maxBot), (minLeft, maxBot), (0,0,255), 3)
cv2.line(resized, (maxRight, minTop), (minLeft, minTop), (0,0,255), 3)

cv2.imwrite("contour.jpg", resized)



