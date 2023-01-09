#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

def obtain_connected_region_stats(i, stats, centroids):
    
    connected_region_stats = {}
    connected_region_stats['x'] = stats[i, cv2.CC_STAT_LEFT]
    connected_region_stats['y'] = stats[i, cv2.CC_STAT_TOP]
    connected_region_stats['w'] = stats[i, cv2.CC_STAT_WIDTH]
    connected_region_stats['h'] = stats[i, cv2.CC_STAT_HEIGHT]
    connected_region_stats['area'] = stats[i, cv2.CC_STAT_AREA]
    connected_region_stats['center'] = centroids[i]

    return connected_region_stats

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

    # Apply Gaussian Filter
    gaussian = cv2.GaussianBlur(gray, (3,3), 0)

    # Threshold the image
    adaptive_img = cv2.adaptiveThreshold(gaussian, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 15, 3)
    drawImg = cv2.cvtColor(adaptive_img, cv2.COLOR_GRAY2BGR)

    showImage(drawImg)


    # Morphology open
    kernel = np.ones((5,5),np.uint8)
    opening = cv2.morphologyEx(adaptive_img, cv2.MORPH_CLOSE, kernel)
    drawImg = cv2.cvtColor(opening, cv2.COLOR_GRAY2BGR)

    # Find Contours
    numLabels, labels, stats, centroids = cv2.connectedComponentsWithStats(opening, 8, cv2.CV_32S)

    # Find the bounding box coordinates of each contour
    boundingBoxPoints = find_bounding_box_coords_from_contours(centroids, stats, Point(resized.shape[1], resized.shape[0], None))
    
    # Draw the bounding boxes
    resized = draw_bounding_boxes(resized, boundingBoxPoints)

    # Store the boundingBoxPoints in marker
    boundingBoxMarker = store_boundingBoxPoints_in_marker(boundingBoxPoints)

    

    image_message = bridge.cv2_to_imgmsg(resized, encoding="passthrough")
    
    image_pub = rospy.Publisher('armCamera/segmentedBlobs_AnnotatedImage', Image, queue_size=10)
    image_pub.publish(image_message)

    rawimage_pub = rospy.Publisher('armCamera/segmentedBlobs_RawImage', Image, queue_size=10)
    rawimage_pub.publish(msg)

    marker_pub = rospy.Publisher("armCamera/segmentedBlobs_BoundingBoxPoints", Marker, queue_size = 10)
    marker_pub.publish(boundingBoxMarker)
    start_node.rate.sleep()

def start_node():
    rospy.init_node('segmented_colour')
    rospy.loginfo('segmented_colour node started')
    start_node.rate = rospy.Rate(2)
    rospy.Subscriber("/armCamera/color/image_raw", Image, process_image)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()

    except rospy.ROSInterruptException:
        pass