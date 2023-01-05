#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

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

def find_bounding_box_coords_from_contours(contours, maxPoint):

    boundingBoxPoints = []

    for contour in contours:
        bottomRight = Point(0, 0, None)
        topLeft = Point(sys.maxint, sys.maxint, sys.maxint)
        
        # Find the max bounding box coords for each contour
        for point in contour:
            point = point[0]
            if topLeft.x > point[0]:
                topLeft.x = point[0] - 20
            if topLeft.y > point[1]:
                topLeft.y = point[1] - 20

            if bottomRight.x < point[0]:
                bottomRight.x = point[0] + 20
            if bottomRight.y < point[1]:
                bottomRight.y = point[1] + 20

        if (topLeft.x < 0):
            topLeft.x = 0
        if (topLeft.y < 0):
            topLeft.y = 0
        if (bottomRight.x > maxPoint.x):
            bottomRight.x = maxPoint.x
        if (bottomRight.y > maxPoint.y):
            bottomRight.y = maxPoint.y

        boundingBoxPoints.append([topLeft, bottomRight])
    
    return boundingBoxPoints

def find_contours(image):
    min_area = 400
    accepted_contours = []
    
    _, contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        if cv2.contourArea(contour) > min_area:
            accepted_contours.append(contour)

    return accepted_contours

def draw_bounding_boxes(input_image, boundingBoxPoints):
    
    image = input_image.copy()
    for boundingBoxLocation in boundingBoxPoints:
        image = cv2.rectangle(image, (boundingBoxLocation[0].x, boundingBoxLocation[0].y), (boundingBoxLocation[1].x, boundingBoxLocation[1].y), (255, 0, 0), 3)

    return image

def store_boundingBoxPoints_in_marker(boundingBoxPoints):
    
    boundingBoxPointsArray = []

    for boundingBoxLocation in boundingBoxPoints:
        boundingBoxPointsArray.append(boundingBoxLocation[0])
        boundingBoxPointsArray.append(boundingBoxLocation[1])
    
    marker = Marker()
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.points = boundingBoxPointsArray

    return marker

def process_image(msg):
    #Resize params
    resize_x = 1
    resize_y = 1

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
    contours = find_contours(opening)

    # Find the bounding box coordinates of each contour
    boundingBoxPoints = find_bounding_box_coords_from_contours(contours, Point(resized.shape[1], resized.shape[0], None))
    
    # Draw the bounding boxes
    resized = draw_bounding_boxes(resized, boundingBoxPoints)

    # Store the boundingBoxPoints in marker
    boundingBoxMarker = store_boundingBoxPoints_in_marker(boundingBoxPoints)

    showImage(resized)

    image_message = bridge.cv2_to_imgmsg(resized, encoding="passthrough")
    
    image_pub = rospy.Publisher('armCamera/nearestColourBlob', Image, queue_size=10)
    image_pub.publish(image_message)

    marker_pub = rospy.Publisher("armCamera/colourBlobBoundingBoxPoints", Marker, queue_size = 2)
    marker_pub.publish(boundingBoxMarker)

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