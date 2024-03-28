import cv2
import numpy as np
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
def color_clahe(img):
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    cl = clahe.apply(l)
    limg = cv2.merge((cl, a, b))
    final = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    return final

def detect_orange_ball(image):
    aspect_ratio = image.shape[1] / image.shape[0]
    image= cv2.resize(image, (int(600 * aspect_ratio), 600))
    image = color_clahe(image)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([0, 100, 100])
    upper_orange = np.array([20, 255, 255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    mask = cv2.dilate(mask, None, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, None,iterations=10)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
    for contour in contours:
        M = cv2.moments(contour) 
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(image, (cX, cY), 7, (255, 255, 255), -1)
        x = 2 *(M["m10"] / M["m00"]) / image.shape[1]-1
        print(x)

def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    detect_orange_ball(image)

def main():
    rospy.init_node('orange_cap_node')
    image_sub = rospy.Subscriber('/oakd/image/raw', Image, image_callback)
    int_pub = rospy.Publisher('/oakd/image/orange', Int32, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()
