from styx_msgs.msg import TrafficLight
import cv2
import numpy as np
import os
import tensorflow as tf
import rospy

USE_CNN = True

class TLClassifier(object):
    def __init__(self, sim=True):
        #TODO load classifier
        self.RED_LOWER_LIMIT = np.array([0,50,50]) # in HSV
        self.RED_UPPER_LIMIT = np.array([10,255,255]) # in HSV
        self.RED2_LOWER_LIMIT = np.array([160,50,50]) # in HSV
        self.RED2_UPPER_LIMIT = np.array([179,255,255]) # in HSV
        self.circle_stat = False
        self.traffic_stat = TrafficLight.UNKNOWN
        if USE_CNN:
            #setup model paths
            self.current_dir = os.path.dirname(os.path.realpath(__file__))
            if sim:
                self.MODEL_PATH = 'model/sim_model/frozen_inference_graph.pb'
            else:
                self.MODEL_PATH = 'model/real_model/frozen_inference_graph.pb'
            self.PATH_2_CKPT = os.path.join(self.current_dir, self.MODEL_PATH)
            # Intialise the Graphs
            self.detection_graph = tf.Graph()

            self.category_index = {1: {'id': 1, 'name': 'Green'},
                                   2: {'id': 2, 'name': 'Red'},
                                   3: {'id': 3, 'name': 'Yellow'},
                                   4: {'id': 4, 'name': 'off'}}
            self._initialise_cnn_model()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
        image (cv::Mat): image containing the traffic light

        Returns:
        int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        self.traffic_stat = TrafficLight.UNKNOWN

        if USE_CNN:
            self.traffic_stat = self.classify_by_cnn(image)
        else:
            self.traffic_stat = self.classify_by_cv(image)

        return traffic_stat


    def classify_by_cv(self, image):
        '''
        method to classify traffic light by open cv
        returns a light state
        '''
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

    def classify_by_cnn(self, image):
        '''
        Classify based on cnn
        '''
        traffic_stat = TrafficLight.UNKNOWN
        # Prepare the image
        # Copy image
        new_image = image.copy()

        # images are already in height x width x 3
        # convert to RGB
        image_rgb = cv2.cvtColor(new_image, cv2.COLOR_BGR2RGB)

        # convert to add additional dimension
        image_np_expanded = np.expand_dims(image_rgb, axis=0)

        # detect
        (boxes, scores, classes, num) = self.session.run([self.detection_boxes,
                                                          self.detection_scores,
                                                          self.detection_classes,
                                                          self.num_detections],
                                                          feed_dict={self.image_tensor: image_np_expanded})
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        min_score_thresh = .50
        for i in range(boxes.shape[0]):
            if scores is None or scores[i] > min_score_thresh:
                class_name = self.category_index[classes[i]]['name']
                if class_name == 'Red':
                    traffic_stat = TrafficLight.RED
                else:
                    traffic_stat = TrafficLight.UNKNOWN
        return traffic_stat

    def _initialise_cnn_model(self):
        '''
        Initialise cnn model
        '''
        try:
            config = tf.ConfigProto()
            config.gpu_options.allow_growth = True

            self.detection_graph = tf.Graph()

            with self.detection_graph.as_default():
                ObjDet_graph_def = tf.GraphDef()

                with tf.gfile.GFile(self.PATH_2_CKPT, 'rb') as fid:
                    serialized_graph = fid.read()
                    ObjDet_graph_def.ParseFromString(serialized_graph)
                    tf.import_graph_def(ObjDet_graph_def, name='')

                self.session = tf.Session(graph=self.detection_graph, config=config)

            # Definite input and output Tensors for detection_graph
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

            # Each box represents a part of the image where a particular object was detected.
            self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
        except Exception as err:
            rospy.loginfo(str(err))
            global USE_CNN
            USE_CNN = False
            rospy.loginfo("Unable to Initialise CNN Model ! Switching to Open CV Classifier")
