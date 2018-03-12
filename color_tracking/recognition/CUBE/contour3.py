import numpy as np
import math
import cv2

# LOAD the image and show it
import imutils

image = cv2.imread("cube51.jpg")
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
rotated = imutils.rotate_bound(resized, 90)
# cv2.imshow("Rotated (Problematic)", rotated)
# cv2.waitKey(0)

# resized = image
# cv2.imshow("resized", resized)
# cv2.waitKey(0)

# PROCESSING the image
# convert the image to grayscale, blur it, and find edges
gray = cv2.cvtColor(rotated, cv2.COLOR_BGR2GRAY)
gray = cv2.bilateralFilter(gray, 11, 17, 17)
edged = cv2.Canny(gray, 30, 200)
# cv2.imshow("resized", edged)
# cv2.waitKey(0)

# find the contours
_, contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = sorted(contours, key=cv2.contourArea, reverse=True)[:5]
# cv2.drawContours(resized, [cnts[1]], -1, (0, 255, 0), 3)
# cv2.imwrite("contour.jpg", resized)
screenCnt = cnts[0]  # the largest countour is most likely the stack of cubes

# CROP make bounding rectangle around stack and crop out
# x,y,w,h = cv2.boundingRect(screenCnt)
# img = cv2.rectangle(resized,(x,y),(x+w,y+h),(0,255,0),2)
# crop_img = img[y:y+h, x:x+w]
crop_img = rotated
# cv2.imshow("cropped", crop_img)
# cv2.waitKey(0)

# ANALYZE find lines in the newly cropped area
gray2 = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
edged2 = cv2.Canny(gray2, 50, 150, apertureSize=3)
# cv2.imshow("cropped", edged2)
# cv2.waitKey(0)
# minLineLength - Minimum length of line. Line segments shorter than this are rejected.
# maxLineGap - Maximum allowed gap between line segments to treat them as single line.

# store the left and right equations with (m, b) format
left_eqn = []
right_eqn = []

for i in range(10, 70):
    theta = math.pi * i / 180
    lines = cv2.HoughLinesP(edged, 1, theta, 2, 80, 40, 1)
    # lines = cv2.HoughLines(edged, 1, np.pi / 180, 200)
    if lines is not None and lines[0] is not None:
        for i in range(0, len(lines)):

            line = lines[i][0]

            pt1 = (line[0],line[1])
            pt2 = (line[2],line[3])

            angle = np.arctan2(line[3] - line[1], line[2] - line[0]) * 180. / np.pi
            # print(angle)
            if angle != 90 and angle != -90 and 60 > angle > -60:
                slope = float(float((line[3] - line[1])) / float((line[2] - line[0])))
                b = line[3] - (slope * line[2])
                if slope < 0:
                    left_eqn.append((slope, b))
                elif slope > 0:
                    right_eqn.append((slope, b))
                # print("slope", slope, "b", b, line[3] - line[1])
                # pt3 = (1000, int((slope * 1000 + b)))
                # pt4 = (-1000, int((slope * -1000 + b)))
                cv2.line(rotated, pt1, pt2, (0,0,255), 3)


# for i in range(10, 70):
# angle = np.pi * 1 / 180
# lines = cv2.HoughLines(edged, 1, angle, 55)
# if lines is not None:
#     # print(lines)
#     for line in lines:
#         for rho, theta in line:
#             # print(theta)
#             if float(np.pi * 60 / 180) > theta > float(np.pi * 10 / 180):
#                 a = np.cos(theta)
#                 b = np.sin(theta)
#                 x0 = a * rho
#                 y0 = b * rho
#                 x1 = int(x0 + 1000 * (-b))
#                 y1 = int(y0 + 1000 * (a))
#                 x2 = int(x0 - 1000 * (-b))
#                 y2 = int(y0 - 1000 * (a))
#
#                 slope = float(float((y2 - y1)) / float((x2 - x1)))
#                 b = y2 - (slope * x2)
#                 if slope < 0:
#                     left_eqn.append((slope, b))
#                 elif slope > 0:
#                     right_eqn.append((slope, b))
#                 cv2.line(resized, (x1, y1), (x2, y2), (0, 0, 255), 2)

print(len(left_eqn), left_eqn)
print(len(right_eqn), right_eqn)
cv2.imwrite("res52.jpg", rotated)
