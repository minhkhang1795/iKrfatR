import numpy as np
import matplotlib.pyplot as plt
import cv2

# load the image and show it
image = cv2.imread("cube1.JPG")
# cv2.imshow("original", image)
# cv2.waitKey(0)

# we need to keep in mind aspect ratio so the image does
# not look skewed or distorted -- therefore, we calculate
# the ratio of the new image to the old image
r = 800.0 / image.shape[1]
dim = (800, int(image.shape[0] * r))

# perform the actual resizing of the image and show it
resized = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)
# cv2.imshow("resized", resized)
# cv2.waitKey(0)

# convert the image to grayscale, blur it, and find edges
# in the image
gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
gray = cv2.bilateralFilter(gray, 11, 17, 17)
edged = cv2.Canny(gray, 30, 200)
# cv2.imshow("resized", edged)
# cv2.waitKey(0)

# find contours in the edged image, keep only the largest
# ones, and initialize our screen contour
ret3, thresh = cv2.threshold(edged, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
# cv2.imshow("thresh", thresh)
# cv2.waitKey(0)

minLineLength = 500
maxLineGap = 0
i = 10
while i < 60:
    lines = cv2.HoughLinesP(thresh, 1, (i * np.pi / 180), 100, minLineLength, maxLineGap)
    #lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)
    #print(lines)
    if lines is not None:
        print(lines)
        for x in range(0, len(lines)):
            for x1, y1, x2, y2 in lines[x]:
                cv2.line(resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.imshow('line', resized)
        cv2.waitKey(1000)
    i = i + 1

# _, contours, _= cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# cnts = sorted(contours, key = cv2.contourArea, reverse = True)[:3]
# screenCnt = None
#
# # loop over our contours
#
# for c in cnts:
#     cv2.drawContours(resized, c, -1, (0, 255, 0), 3)
#     cv2.imshow("Game Boy Screen", resized)
#     cv2.waitKey(2000)
#     # approximate the contour
#     peri = cv2.arcLength(c, True)
#     approx = cv2.approxPolyDP(c, 0.02 * peri, True)
#
#     # if our approximated contour has four points, then
#     # we can assume that we have found our screen
#     if len(approx) == 4:
#         screenCnt = approx
#         print("four corners found")
#         break

# cv2.drawContours(resized, [cnts[0]], -1, (0, 255, 0), 3)

