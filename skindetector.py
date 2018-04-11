# USAGE
# python skindetector.py
# python skindetector.py --video video/skin_example.mov

# import the necessary packages
from pyimagesearch import imutils
import numpy as np
import argparse
import cv2

def has_hand(image, image_path="result.JPG"):

    # define the upper and lower boundaries of the HSV pixel
    # intensities to be considered 'skin'
    lower = np.array([0, 48, 80], dtype="uint8")
    upper = np.array([20, 255, 255], dtype="uint8")

    # resize the frame, convert it to the HSV color space,
    # and determine the HSV pixel intensities that fall into
    # the speicifed upper and lower boundaries
    frame = imutils.resize(image, width=400)
    converted = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    skinMask = cv2.inRange(converted, lower, upper)

    # apply a series of erosions and dilations to the mask
    # using an elliptical kernel
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
    skinMask = cv2.erode(skinMask, kernel, iterations=2)
    skinMask = cv2.dilate(skinMask, kernel, iterations=2)

    # blur the mask to help remove noise, then apply the
    # mask to the frame
    skinMask = cv2.GaussianBlur(skinMask, (3, 3), 0)
    skin = cv2.bitwise_and(frame, frame, mask=skinMask)

    # show the skin in the image along with the mask
    cv2.imwrite(image_path[:-4]+"_2.JPG", np.hstack([frame, skin]))
    count = 0
    for array in skinMask:
        for elm in array:
            if elm == 255:
                # print("has skin")
                count += 1

    print(count)

    if count > 1500:
        print("has skin")
        return True


    else:
        return False

if __name__ == '__main__':
    image_path = "test_images/hand6.JPG"
    image = cv2.imread(image_path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    print("hello", image)
    has_hand(image, image_path)


