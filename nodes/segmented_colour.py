#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

from geometry_msgs.msg import Point

window = 50
x_history = [0] * window
y_history = [0] * window
count = 0

def moving_average(coord):
    global count 
    count += 1
    if count > window:
    
        x = coord.x
        y = coord.y

        x_history.pop()
        x_history.insert(0, x)

        y_history.pop()
        y_history.insert(0, y)

        return Point(int(np.mean(x_history)), int(np.mean(y_history)), None)
    else:
        return Point(-1, -1, -1)



def find_nearest_contour(contours, image):
    
    # Copy the image to prevent modification
    image = image.copy()
    
    # Declare initial params to find the nearest contour
    max_y = 0
    nearest_contour_id = -1
    contour_center_coord = Point(-1, -1, -1)

    # Min area of the blob, obtained empirically
    min_area = 400

    # if we find contours
    if len(contours) > 0:
        for i in range(0, len(contours)):            
            c = contours[i]

            if cv2.contourArea(c) > min_area:
                M = cv2.moments(c)

                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0

                if cY > max_y:
                    max_y = cY
                    nearest_contour_id = i
                    contour_center_coord = Point(cX, cY, None)
    else:
        return image, None, None

    
    if nearest_contour_id != -1:
        print("Area: " + str(cv2.contourArea(contours[nearest_contour_id])) + "\nCentroid: " +  str(contour_center_coord) + "\n")
        cv2.drawContours(image, contours, nearest_contour_id, (0, 255, 0), 3)
    
    contour_center_coord = moving_average(contour_center_coord)
    return image, nearest_contour_id, contour_center_coord

def find_offset_from_control_point(coord, control_point):
    # Camera need to Move right: +ve x || Camera need to Move up: +ve y
    offset = Point(control_point.x-coord.x, -1*(control_point.y-coord.y), None)
    print ("Offset: " + str(offset) + "\n")
    return offset

def showImage(img):
    cv2.imshow('image', img)
    cv2.waitKey(1)

def process_image(msg):
    print("===")
    #Resize params
    resize_x = 0.5
    resize_y = 0.5

    # Declare the cvBridge object
    bridge = CvBridge()
    orig = bridge.imgmsg_to_cv2(msg, "bgr8")
    drawImg = orig
    
    # Resize the image 
    resized = cv2.resize(orig, None, fx=resize_x, fy=resize_y)
    drawImg = resized
    
    # Convert to grayscale
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    drawImg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    # Threshold the image
    threshVal = 75
    ret,thresh = cv2.threshold(gray, threshVal, 255, cv2.THRESH_BINARY_INV)
    drawImg = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

    # Morphology open
    kernel = np.ones((5,5),np.uint8)
    opening = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    drawImg = cv2.cvtColor(opening, cv2.COLOR_GRAY2BGR)

    # Find Contours
    _, contours, hierarchy = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the nearest contour
    image, nearest_contour_id, contour_center_coord = find_nearest_contour(contours, resized)
    
    if (~(nearest_contour_id is None)):
        # Set a control point
        control_point = Point(image.shape[1]*resize_x, image.shape[0]*resize_y, None)
        offset_from_control_point = find_offset_from_control_point(control_point, contour_center_coord)

        # Visualise the control point and centroid
        image = cv2.circle(image, (int(contour_center_coord.x), int(contour_center_coord.y)), 3, color=(0, 0, 255), thickness=3)
        image = cv2.circle(image, (int(control_point.x), int(control_point.y)), 3, color=(0, 255, 0), thickness=3)

        showImage(image)

        pub_image = rospy.Publisher('armCamera/nearest_colourBlob', Image, queue_size=10)
        pub_centroid = rospy.Publisher('armCamera/nearest_colourBlobCenter', Point, queue_size=10)

        image_message = bridge.cv2_to_imgmsg(image, encoding="passthrough")
        
        pub_image.publish(image_message)
        pub_centroid.publish(offset_from_control_point)

def start_node():
    rospy.init_node('segmented_colour')
    rospy.loginfo('segmented_colour node started')

    rospy.Subscriber("/armCamera/color/image_raw", Image, process_image)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass