#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Float64MultiArray  

window = 50
x_history = [0] * window
y_history = [0] * window
count = 0

def moving_average(point):
    global count 
    count += 1
    if count > window:
    
        x = point[0]
        y = point[1]

        x_history.pop()
        x_history.insert(0, x)

        y_history.pop()
        y_history.insert(0, y)

        return [np.mean(x_history), np.mean(y_history)]
    else:
        print(count)
        return [-1, -1]



def find_nearest_contour(contours, image):
    
    image = image.copy()
    
    max_y = 0
    nearest_contour_id = -1
    contour_center_coord = [-1, -1]

    min_area = 400

    if len(contours) > 0:
        for i in range(0, len(contours)):            
            c = contours[i]

            if cv2.contourArea(c) > min_area:
                M = cv2.moments(c)

                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    # set values as what you need in the situation
                    cX, cY = 0, 0

                if cY > max_y:
                    max_y = cY
                    nearest_contour_id = i
                    contour_center_coord = [cX, cY]

    
    if nearest_contour_id != -1:
        print("Area: " + str(cv2.contourArea(contours[nearest_contour_id])) + " Centroid: " +  str(contour_center_coord))
        cv2.drawContours(image, contours, nearest_contour_id, (0, 255, 0), 3)
    
    contour_center_coord = moving_average(contour_center_coord)
    print(contour_center_coord)
    return image, nearest_contour_id, contour_center_coord

def offset_from_control_point(coord, control_point):
    return [float(control_point[0] - coord[0]),float(control_point[1] - coord[1])]

def showImage(img):
    cv2.imshow('image', img)
    cv2.waitKey(1)

def process_image(msg):
    
    # Declare the cvBridge object
    bridge = CvBridge()
    orig = bridge.imgmsg_to_cv2(msg, "bgr8")
    drawImg = orig
    
    # Resize the image 
    resized = cv2.resize(orig, None, fx=0.5, fy=0.5)
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
    
    # Set a control point
    control_point = [image.shape[1]/2, image.shape[0]/2]
    amt_to_move = offset_from_control_point(contour_center_coord, control_point)

    # Visualise the control point and centroid
    image = cv2.circle(image, (int(contour_center_coord[0]), int(contour_center_coord[1])), 3, color=(0, 0, 255), thickness=3)
    image = cv2.circle(image, (int(control_point[0]), int(control_point[1])), 3, color=(0, 255, 0), thickness=3)

    showImage(image)

    pub_image = rospy.Publisher('segmented_colour/image', Image, queue_size=10)
    pub_centroid = rospy.Publisher('segmented_colour/centroid', Float64MultiArray, queue_size=10)

    image_message = bridge.cv2_to_imgmsg(image, encoding="passthrough")
    pub_image.publish(image_message)

    my_msg = Float64MultiArray()  
    my_msg.data = amt_to_move
    pub_centroid.publish(my_msg)

    try:
       pass
    except Exception as err:
        print err

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