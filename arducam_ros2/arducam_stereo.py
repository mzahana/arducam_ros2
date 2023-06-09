#!/usr/bin/env python


###################################################
###################################################
#        !!!!!WARNNING!!!!!
#       THIS IS A ROS 1 NODE
#       IT NEEDS TO BE CONVERTED TO A ROS 2 NODE
###################################################
###################################################


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml

import cv2
import numpy as np
from datetime import datetime
import array
import fcntl
import os
import argparse
from .utils import ArducamUtils

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
# from camera_info_manager import CameraInfoManager # !!! ROS 1 !!!

class ArduCamNode(Node):
    def __init__(self):
        super().__init__('arducam_node')
        # Get left camera params
        self.declare_parameter('left_cam_info_file', 'left_cam_info.yaml')
        self._left_cam_info_file = self.get_parameter('left_cam_info_file').get_parameter_value().string_value

        # Get right camera params
        self.declare_parameter('right_cam_info_file', 'right_cam_info.yaml')
        self._right_cam_info_file = self.get_parameter('right_cam_info_file').get_parameter_value().string_value

        self.declare_parameter('device', 0)
        self._device = self.get_parameter('right_cam_info_file').get_parameter_value().integer_value

        self.declare_parameter('pixelformat', 'GREY')
        self._pixelformat = self.get_parameter('pixelformat').get_parameter_value().string_value

        self.declare_parameter('width', 1280)
        self._width = self.get_parameter('width').get_parameter_value().integer_value

        self.declare_parameter('height', 400)
        self._height = self.get_parameter('height').get_parameter_value().integer_value

        self.declare_parameter('frame_id', 'cam0')
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        

        # flags to publish cam info if loaded
        self._pub_caminfo = True

        self._left_cam_info_msg = CameraInfo()
        self._right_cam_info_msg = CameraInfo()

        # Cvbridge
        self._cv_bridge = CvBridge()
        
        # open camera
        self._cap = cv2.VideoCapture(self._device, cv2.CAP_V4L2)
        # set width
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        # set height
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)

        self._arducam_utils = ArducamUtils(self._device)

        self.show_info(self._arducam_utils)

        # turn off RGB conversion
        if self._arducam_utils.convert2rgb == 0:
            self._cap.set(cv2.CAP_PROP_CONVERT_RGB, self._arducam_utils.convert2rgb)
        
        
        # Load camera info from YAML files
        # If failed, camInfo msgs will not be published
        self.load_camera_info()

        # Publishers
        # CamInfo
        self._left_cam_info_pub = self.create_publisher(CameraInfo, 'stereo/left/camera_info', 10)
        self._right_cam_info_pub = self.create_publisher(CameraInfo, 'stereo/right/camera_info', 10)
        # Images
        self._left_img_pub = self.create_publisher(Image, 'stereo/left/image_raw', 10)
        self._right_img_pub = self.create_publisher(Image, 'stereo/right/image_raw', 10)

        # TODO We need to create a timer for the run() function
        fps = 0.01  # seconds. WARNING. this is limited by the actual camera FPS
        self._cam_timer = self.create_timer(fps, self.run)

        


    def load_camera_info(self):
        try:

            with open(self._left_cam_info_file, 'r') as file:
                left_cam_data = yaml.safe_load(file)
        except Exception as e:
            self.get_logger().warn("[ArduCamNode::load_camera_info] Could not load left camera parameters")
            self._pub_caminfo = False
            return
        
        try:

            with open(self._right_cam_info_file, 'r') as file:
                right_cam_data = yaml.safe_load(file)
        except Exception as e:
            self.get_logger().warn("[ArduCamNode::load_camera_info] Could not load right camera parameters")
            self._pub_caminfo = False
            return

        self._left_cam_info_msg.width = left_cam_data['image_width']
        self._left_cam_info_msg.height = left_cam_data['image_height']
        self._left_cam_info_msg.distortion_model = left_cam_data['distortion_model']
        self._left_cam_info_msg.K = left_cam_data['camera_matrix']['data']
        self._left_cam_info_msg.D = left_cam_data['distortion_coefficients']['data']
        self._left_cam_info_msg.R = left_cam_data['rectification_matrix']['data']
        self._left_cam_info_msg.P = left_cam_data['projection_matrix']['data']

        # self.camera_info_publisher.publish(camera_info_msg)
        self.get_logger().info('Loaded left camera info from %s', self._left_cam_info_file)

        self._right_cam_info_msg.width = right_cam_data['image_width']
        self._right_cam_info_msg.height = right_cam_data['image_height']
        self._right_cam_info_msg.distortion_model = right_cam_data['distortion_model']
        self._right_cam_info_msg.K = right_cam_data['camera_matrix']['data']
        self._right_cam_info_msg.D = right_cam_data['distortion_coefficients']['data']
        self._right_cam_info_msg.R = right_cam_data['rectification_matrix']['data']
        self._right_cam_info_msg.P = right_cam_data['projection_matrix']['data']

        self.get_logger().info('Loaded right camera info from %s', self._right_cam_info_file)


    def resize(self, frame, dst_width):
        width = frame.shape[1]
        height = frame.shape[0]
        scale = dst_width * 1.0 / width
        return cv2.resize(frame, (int(scale * width), int(scale * height)))

    def run(self):
        ret, frame = self._cap.read()
        if self._arducam_utils.convert2rgb == 0:
            w = self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            h = self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            frame = frame.reshape(int(h), int(w))

        frame = self._arducam_utils.convert(frame)

        encoding = "bgr8" if len(frame.shape) == 3 and frame.shape[2] >= 3 else "mono8"

        width = frame.shape[1]
        # height = frame.shape[0]

        left_img = frame[:, :width//2]
        right_img = frame[:, width//2:]

        left_img_msg = self._cv_bridge.cv2_to_imgmsg(left_img, encoding)
        left_img_msg.header.frame_id = self._frame_id

        right_img_msg = self._cv_bridge.cv2_to_imgmsg(right_img, encoding)
        right_img_msg.header.frame_id = self._frame_id

        capture_time = self.get_clock().now().to_msg()

        left_img_msg.header.stamp = capture_time
        right_img_msg.header.stamp = capture_time

        self._left_img_pub.publish(left_img_msg)
        self._right_img_pub.publish(right_img_msg)
        
        if self._pub_caminfo:
            # Update camerainfo
            self._left_cam_info_msg.header.stamp = capture_time
            self._right_cam_info_msg.header.stamp = capture_time

            self._left_cam_info_msg.header.frame_id = self._frame_id
            self._right_cam_info_msg.header.frame_id = self._frame_id
            self._left_cam_info_pub.publish(self._left_cam_info_msg)
            self._right_cam_info_pub.publish(self._righ)
            


    def fourcc(self, a, b, c, d):
        return ord(a) | (ord(b) << 8) | (ord(c) << 16) | (ord(d) << 24)

    def pixelformat(self, string):
        if len(string) != 3 and len(string) != 4:
            msg = "{} is not a pixel format".format(string)
            raise argparse.ArgumentTypeError(msg)
        if len(string) == 3:
            return self.fourcc(string[0], string[1], string[2], ' ')
        else:
            return self.fourcc(string[0], string[1], string[2], string[3])

    def show_info(self, arducam_utils):
        _, firmware_version = arducam_utils.read_dev(ArducamUtils.FIRMWARE_VERSION_REG)
        _, sensor_id = arducam_utils.read_dev(ArducamUtils.FIRMWARE_SENSOR_ID_REG)
        _, serial_number = arducam_utils.read_dev(ArducamUtils.SERIAL_NUMBER_REG)
        print("Firmware Version: {}".format(firmware_version))
        print("Sensor ID: 0x{:04X}".format(sensor_id))
        print("Serial Number: 0x{:08X}".format(serial_number))

def main(args=None):
    rclpy.init(args=args)
    arducam_node = ArduCamNode()
    arducam_node.get_logger().info("arducam_node has started")
    rclpy.spin(arducam_node)
    arducam_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()