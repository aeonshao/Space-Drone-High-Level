'''
ArUco tool detector node

Subscribes to the downward camera, finds ArUco tags (each tag = one tool), estimates
their pose relative to the camera and publishes them as a ToolDetectionArray.
The C++ tool_registry node turns these into map positions using the drone pose.

Topics:
    in:  /camera/image_raw      (sensor_msgs/Image)
         /camera/camera_info    (sensor_msgs/CameraInfo)
    out: /spacedrone/tool_detections   (spacedrone_interfaces/ToolDetectionArray)
         /spacedrone/debug_image       (sensor_msgs/Image) only if someone is subscribed

This replaced the old aruco_publisher.py in yolo_pub which opened the camera itself
inside a timer callback (which blocked spin forever lol).
'''

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge

from spacedrone_interfaces.msg import ToolDetection, ToolDetectionArray
from spacedrone_vision.aruco_utils import (TagDetector, pose_estimation, rvec_to_quaternion,
                                           tag_pixel_area, intrinsics_from_fov)


class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.5)
        self.declare_parameter('tool_ids', [0, 3, 7, 12])
        self.declare_parameter('tool_names', ['wrench', 'hammer', 'drill', 'toolbox'])
        self.declare_parameter('ignore_unknown_ids', True)
        self.declare_parameter('process_every_n', 2)
        self.declare_parameter('max_distance', 15.0)
        self.declare_parameter('fallback_hfov', 1.74)  # mono_cam in PX4 gz models
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')

        self.marker_size = self.get_parameter('marker_size').value
        self.ignore_unknown = self.get_parameter('ignore_unknown_ids').value
        self.every_n = max(1, self.get_parameter('process_every_n').value)
        self.max_distance = self.get_parameter('max_distance').value
        self.fallback_hfov = self.get_parameter('fallback_hfov').value

        ids = self.get_parameter('tool_ids').value
        names = self.get_parameter('tool_names').value
        if len(ids) != len(names):
            self.get_logger().error('tool_ids and tool_names are different lengths!! check tools.yaml')
        self.tool_names = dict(zip(ids, names))

        self.detector = TagDetector(self.get_parameter('aruco_dict').value)
        self.bridge = CvBridge()

        # camera calibration, gets filled from camera_info
        self.camera_matrix = None
        self.dist_coeffs = np.zeros(5)
        self.warned_no_info = False

        # #calibration values for the phone camera we used before the sim (from camera calibration/)
        # self.camera_matrix = np.array(((1484.001664055822, 0, 945.7762352668258),
        #                                (0, 1484.001664055822, 520.8557474898012),
        #                                (0, 0, 1)))
        # self.dist_coeffs = np.array((-0.0179606305852, 0.180716809921, -0.00015874041681,
        #                              -0.00190823724612, -0.333257089362))

        image_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value

        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data)

        self.det_pub = self.create_publisher(ToolDetectionArray, '/spacedrone/tool_detections', 10)
        self.debug_pub = self.create_publisher(Image, '/spacedrone/debug_image', 1)

        self.frame_count = 0
        self.seen_ids = set()

        self.get_logger().info(f'aruco detector up, looking for tools: {self.tool_names}')

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().info(f'got camera_info {msg.width}x{msg.height}')
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if len(msg.d) > 0:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)

        # dont need it after the first one, camera doesnt change
        # self.destroy_subscription(self.info_sub)
        # ^ this crashed with "subscription is not valid" sometimes, just leaving it

    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.every_n != 0:
            return

        # frame = self.bridge.imgmsg_to_cv2(msg)   # gazebo sends rgb8 so the colors were swapped in the debug view
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        if self.camera_matrix is None:
            if not self.warned_no_info:
                self.get_logger().warn('no camera_info yet, using intrinsics from FOV param')
                self.warned_no_info = True
            K = intrinsics_from_fov(w, h, self.fallback_hfov)
        else:
            K = self.camera_matrix

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # convert the image to grayscale for detection
        corners, ids, rejected = self.detector.detect(gray)

        out = ToolDetectionArray()
        out.header = msg.header

        if ids is not None:
            ids = ids.flatten()
            rvecs, tvecs, oks = pose_estimation(corners, self.marker_size, K, self.dist_coeffs)

            for i, tag_id in enumerate(ids):
                tag_id = int(tag_id)
                if not oks[i]:
                    continue
                if tag_id not in self.tool_names and self.ignore_unknown:
                    continue

                t = tvecs[i].flatten()
                dist = float(np.linalg.norm(t))
                if dist > self.max_distance:
                    # pose from really far away is garbage
                    continue

                det = ToolDetection()
                det.tag_id = tag_id
                det.tool_name = self.tool_names.get(tag_id, f'unknown_{tag_id}')
                det.pose = Pose()
                det.pose.position.x = float(t[0])
                det.pose.position.y = float(t[1])
                det.pose.position.z = float(t[2])
                qx, qy, qz, qw = rvec_to_quaternion(rvecs[i])
                det.pose.orientation.x = qx
                det.pose.orientation.y = qy
                det.pose.orientation.z = qz
                det.pose.orientation.w = qw
                det.pixel_area = float(tag_pixel_area(corners[i]))
                out.detections.append(det)

                if tag_id not in self.seen_ids:
                    self.seen_ids.add(tag_id)
                    self.get_logger().info(f'NEW TOOL: {det.tool_name} (id {tag_id}) at {dist:.1f}m from camera')

            # print(f"ID: {ids}, Tvec: {tvecs}")

        # publish even if empty so the registry knows we're alive
        self.det_pub.publish(out)

        # debug image is expensive, only draw it if someone is looking (rqt_image_view etc)
        if self.debug_pub.get_subscription_count() > 0:
            self.publish_debug(frame, corners, ids, out, K)

    def publish_debug(self, frame, corners, ids, out, K):
        dbg = frame.copy()
        if ids is not None and len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(dbg, corners, ids.reshape(-1, 1))
        for det in out.detections:
            p = det.pose.position
            # put the name above the tag. project the tag center back into the image
            pt, _ = cv2.projectPoints(np.array([[p.x, p.y, p.z]]), np.zeros(3), np.zeros(3), K, self.dist_coeffs)
            u, v = int(pt[0][0][0]), int(pt[0][0][1])
            cv2.circle(dbg, (u, v), 6, (0, 0, 255), -1)
            cv2.putText(dbg, f'{det.tool_name} {p.z:.1f}m', (u + 10, v - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        msg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
        msg.header = out.header
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
