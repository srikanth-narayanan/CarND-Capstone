from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.RED_LOWER_LIMIT = np.array([0,50,50]) # in HSV
        self.RED_UPPER_LIMIT = np.array([10,255,255]) # in HSV
        self.RED2_LOWER_LIMIT = np.array([160,50,50]) # in HSV
        self.RED2_UPPER_LIMIT = np.array([179,255,255]) # in HSV
        self.circle_stat = False

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
        image (cv::Mat): image containing the traffic light

        Returns:
        int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        traffic_stat = TrafficLight.UNKNOWN
        # Copy image
        new_image = image.copy()
        image_hsv = cv2.cvtColor(new_image, cv2.COLOR_BGR2HSV)

        # Apply Red limits
        red_image = cv2.inRange(image_hsv, self.RED_LOWER_LIMIT, self.RED_UPPER_LIMIT)

        # Apply all between blue and red
        rest_image = cv2.inRange(image_hsv, self.RED2_LOWER_LIMIT, self.RED2_UPPER_LIMIT)

        # Weight the entire image
        weighted_image = cv2.addWeighted(red_image, 1.0, rest_image, 1.0, 0.0)

        blured_image = cv2.GaussianBlur(weighted_image, (15,15), 0.0)

        # Detect red circles using Hough Circle
        image_circles = cv2.HoughCircles(blured_image,cv2.HOUGH_GRADIENT,0.5,41,
                                         param1=70,param2=30,minRadius=5,
                                         maxRadius=150)
        if image_circles is not None:
            traffic_stat = TrafficLight.RED

        return traffic_stat
