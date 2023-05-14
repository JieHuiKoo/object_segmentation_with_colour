#! /usr/bin/python3
import rospy
import sys
import cv2
from sensor_msgs.msg import Image
import numpy as np

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

print("Python Version: " + str(sys.version_info[0]) + '.' + str(sys.version_info[1]))
print("OpenCV Version: " + str(cv2.__version__))

def imgmsg_to_cv2(img_msg):
    rgb8_flag = 0
    if img_msg.encoding != "bgr8":
        rgb8_flag = 1
        # rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()

    if rgb8_flag:
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)

    return image_opencv

def cv2_to_imgmsg(cv_image, encoding):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = encoding
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tobytes()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg

def obtain_connected_region_stats(i, stats, centroids):
    
    connected_region_stats = {}
    connected_region_stats['x'] = stats[i, cv2.CC_STAT_LEFT]
    connected_region_stats['y'] = stats[i, cv2.CC_STAT_TOP]
    connected_region_stats['w'] = stats[i, cv2.CC_STAT_WIDTH]
    connected_region_stats['h'] = stats[i, cv2.CC_STAT_HEIGHT]
    connected_region_stats['area'] = stats[i, cv2.CC_STAT_AREA]
    connected_region_stats['center'] = centroids[i]

    return connected_region_stats

def showImage(img, name, debug):
    if debug:
        cv2.imshow(name, img)
        cv2.waitKey(1)

def find_bounding_box_coords_from_contours(centroids, stats, maxPoint):

    boundingBoxPoints = []

    # The first centroid is the background
    for i in range(1, len(centroids)):
        
        if stats[i, cv2.CC_STAT_AREA] < 500:
            continue
        
        bottomRight = Point(0, 0, None)
        topLeft = Point(np.inf, np.inf, np.inf)
        
        border_margin = 50

        topLeft.x = stats[i, cv2.CC_STAT_LEFT] - border_margin
        topLeft.y = stats[i, cv2.CC_STAT_TOP] - border_margin

        bottomRight.x = stats[i, cv2.CC_STAT_LEFT] + stats[i, cv2.CC_STAT_WIDTH] + border_margin
        bottomRight.y = stats[i, cv2.CC_STAT_TOP]  + stats[i, cv2.CC_STAT_HEIGHT] + border_margin

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

def draw_bounding_boxes(input_image, boundingBoxPoints):
    
    image = input_image.copy()
    for boundingBoxLocation in boundingBoxPoints:
        image = cv2.rectangle(image, (boundingBoxLocation[0].x, boundingBoxLocation[0].y), (boundingBoxLocation[1].x, boundingBoxLocation[1].y), (255, 0, 0), 3)

    return image

def store_boundingBoxPoints_in_marker(boundingBoxPoints):
    
    boundingBoxPointsArray = []

    for boundingBoxLocation in boundingBoxPoints[0:]:
        boundingBoxPointsArray.append(boundingBoxLocation[0])
        boundingBoxPointsArray.append(boundingBoxLocation[1])
    
    marker = Marker()
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.points = boundingBoxPointsArray

    return marker

def draw_claw_mask(input_image):
    height_shift = 10
    width_shift = 30
    claw_offset = 140
    visible_width = 70
    visible_height = 50
    visible_bottom_height = 25

    image = input_image.copy()
    image = cv2.circle(image, (int(image.shape[1]/2)-claw_offset+width_shift, int(image.shape[0]+height_shift)), 100, 0, -1)
    image = cv2.circle(image, (int(image.shape[1]/2)+claw_offset+width_shift, int(image.shape[0]+height_shift)), 100, 0, -1)
    image = cv2.rectangle(image, ((int(image.shape[1]/2-visible_width/2)+width_shift, int(image.shape[0]-visible_height))), ((int(image.shape[1]/2+visible_width/2)+width_shift, int(image.shape[0]))), 255, -1)
    image = cv2.rectangle(image, ((int(image.shape[1]/2-claw_offset/2)+width_shift, int(image.shape[0]-visible_bottom_height))), ((int(image.shape[1]/2+claw_offset/2)+width_shift, int(image.shape[0]))), 0, -1)

    return image

def generate_claw_mask(dim):
    blank_image = np.full(dim, 255, np.uint8)
    claw_mask = draw_claw_mask(blank_image)

    return claw_mask

def process_image(msg):
    debug = 0

    #Resize params
    resize_x = 1
    resize_y = 1

    # Declare the cvBridge object
    orig = imgmsg_to_cv2(msg)
    drawImg = orig
    
    # Resize the image 
    resized = cv2.resize(orig, None, fx=resize_x, fy=resize_y)
    drawImg = resized

    # Convert to grayscale
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    drawImg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    # Apply Gaussian Filter
    gaussian = cv2.GaussianBlur(gray, (3,3), 0)
    showImage(drawImg, 'Gaussian Filter', debug)

    # Threshold the image
    adaptive_img = cv2.adaptiveThreshold(gaussian, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 15, 3)
    drawImg = cv2.cvtColor(adaptive_img, cv2.COLOR_GRAY2BGR)
    showImage(drawImg, 'Thresholded Image', debug)


    # Morphology open
    opening = cv2.morphologyEx(adaptive_img, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))
    close = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, np.ones((11,11),np.uint8))
    drawImg = cv2.cvtColor(close, cv2.COLOR_GRAY2BGR)
    showImage(drawImg, 'Morphology Open', debug)


    # Draw Mask to remove claw
    claw_mask = generate_claw_mask((resized.shape[0], resized.shape[1], 1))
    showImage(claw_mask, 'Claw Mask Image', debug)

    claw_masked_image = cv2.bitwise_and(close, claw_mask)
    showImage(claw_masked_image, 'Claw Masked Image', debug)

    # Find Contours
    numLabels, labels, stats, centroids = cv2.connectedComponentsWithStats(claw_masked_image, 8, cv2.CV_32S)

    # Find the bounding box coordinates of each contour
    boundingBoxPoints = find_bounding_box_coords_from_contours(centroids, stats, Point(resized.shape[1], resized.shape[0], None))
    
    # Draw the bounding boxes
    resized = draw_bounding_boxes(resized, boundingBoxPoints)
    showImage(resized, 'Bounding Boxes', debug)

    # Store the boundingBoxPoints in marker
    boundingBoxMarker = store_boundingBoxPoints_in_marker(boundingBoxPoints)

    image_message = cv2_to_imgmsg(resized, encoding="bgr8")
    
    image_pub = rospy.Publisher('armCamera/segmentedBlobs_AnnotatedImage', Image, queue_size=1)
    image_pub.publish(image_message)

    rawimage_pub = rospy.Publisher('armCamera/segmentedBlobs_RawImage', Image, queue_size=1)
    rawimage_pub.publish(msg)

    marker_pub = rospy.Publisher("armCamera/segmentedBlobs_BoundingBoxPoints", Marker, queue_size = 1)
    marker_pub.publish(boundingBoxMarker)

def start_node():
    rospy.init_node('upload_photo')
    rospy.loginfo('segmented_colour node started')
    rospy.Subscriber("/armCamera/color/image_rect_color/", Image, process_image)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()

    except rospy.ROSInterruptException:
        pass
