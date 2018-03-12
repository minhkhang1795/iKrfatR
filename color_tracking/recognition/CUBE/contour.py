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

_, contours, _= cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = sorted(contours, key = cv2.contourArea, reverse = True)[:1]
screenCnt = cnts[0]

#make bounding rectangle
x,y,w,h = cv2.boundingRect(screenCnt)
img = cv2.rectangle(resized,(x,y),(x+w,y+h),(0,255,0),2)
# cv2.imshow("thresh", img)
# cv2.waitKey(1000)

#crop out the bounding rectangle
crop_img = img[y:y+h, x:x+w]
# cv2.imshow("cropped", crop_img)
# cv2.waitKey(1000)

#find lines in the newly cropped area
gray2 = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
# gray2 = cv2.bilateralFilter(gray, 11, 17, 17)
# edged2 = cv2.Canny(gray, 30, 200)
edged2 = cv2.Canny(gray2,50,150,apertureSize = 3)
# cv2.imshow("cropped", edged2)
# cv2.waitKey(1000)
#ret3, thresh2 = cv2.threshold(edged2, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

i = 1
minLineLength = 20
maxLineGap = 0
lines = cv2.HoughLinesP(edged2,1,np.pi/180,100,minLineLength,maxLineGap)
print(lines)

for x1,y1,x2,y2 in lines[0]:
    cv2.line(crop_img,(x1,y1),(x2,y2),(0,255,0),2)

cv2.imwrite('result.jpg',crop_img)

# while i < 60:
#     print("in the loop")
#     lines = cv2.HoughLinesP(edged2, 1, i * np.pi / 180, 100, minLineLength, maxLineGap)
#     if lines is None:
#         print("it is none")
#     else:
#         for x1, y1, x2, y2 in lines[0]:
#             cv2.line(crop_img, (x1, y1), (x2, y2), (0, 255, 0), 5)
#             cv2.imwrite('houghlines5.jpg', crop_img)
#             cv2.imshow("cropped", crop_img)
#             cv2.waitKey(1000)
#     i = i + 1

# cv2.imwrite('houghlines5.jpg',crop_img)
# cv2.imshow("cropped", crop_img)
# cv2.waitKey(0)
# loop over our contours

for c in cnts:
    # approximate the contour
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.04 * peri, True)
    # cv2.drawContours(resized, approx, -1, (0, 255, 0), 3)
    # cv2.imshow("Game Boy Screen", resized)
    # cv2.waitKey(5000)
    # if our approximated contour has four points, then
    # we can assume that we have found our screen
    # if len(approx) == 4:
    #     screenCnt = approx
    #     print("four corners found")
    #     break

# cv2.drawContours(resized, [cnts[0]], -1, (0, 255, 0), 3)

