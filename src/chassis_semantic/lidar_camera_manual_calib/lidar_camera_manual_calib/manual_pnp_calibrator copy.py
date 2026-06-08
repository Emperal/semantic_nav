import os
import yaml
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R


class ManualPnPCalibrator(Node):
    def __init__(self):
        super().__init__('manual_pnp_calibrator')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('lidar_frame', 'livox_frame')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('save_path', '/tmp/lidar_camera_extrinsic.yaml')

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.clicked_point_topic = self.get_parameter('clicked_point_topic').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.save_path = self.get_parameter('save_path').value

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.camera_matrix = None
        self.dist_coeffs = None

        self.latest_image = None
        self.latest_clicked_3d = None

        self.object_points = []
        self.image_points = []

        self.rvec = None
        self.tvec = None

        self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
        self.create_subscription(PointStamped, self.clicked_point_topic, self.clicked_point_callback, 10)

        self.timer = self.create_timer(0.03, self.visualize)

        cv2.namedWindow('manual_lidar_camera_calib')
        cv2.setMouseCallback('manual_lidar_camera_calib', self.mouse_callback)

        self.get_logger().info('Manual PnP calibrator started.')
        self.get_logger().info('Step: RViz Publish Point first, then click same point in image.')
        self.get_logger().info('Press c: calibrate, s: save, d: delete last pair, q: quit.')

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info(f'Got camera info K={self.camera_matrix.tolist()} D={self.dist_coeffs.tolist()}')

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')

    def clicked_point_callback(self, msg):
        self.latest_clicked_3d = np.array(
            [msg.point.x, msg.point.y, msg.point.z],
            dtype=np.float64
        )
        self.get_logger().info(
            f'Got 3D point in {msg.header.frame_id}: '
            f'{self.latest_clicked_3d.tolist()}'
        )

    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.latest_clicked_3d is None:
            self.get_logger().warn('No 3D point yet. Use Publish Point in RViz first.')
            return

        self.object_points.append(self.latest_clicked_3d.copy())
        self.image_points.append(np.array([x, y], dtype=np.float64))

        self.get_logger().info(
            f'Add pair #{len(self.object_points)}: '
            f'3D={self.latest_clicked_3d.tolist()} 2D=({x}, {y})'
        )

        self.latest_clicked_3d = None

    def run_pnp(self):
        if self.camera_matrix is None:
            self.get_logger().error('No camera_info received.')
            return

        if len(self.object_points) < 6:
            self.get_logger().error('Need at least 6 point pairs.')
            return

        object_points = np.array(self.object_points, dtype=np.float64)
        image_points = np.array(self.image_points, dtype=np.float64)

        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
            reprojectionError=8.0,
            iterationsCount=100
        )

        if not ok:
            self.get_logger().error('solvePnP failed.')
            return

        self.rvec = rvec
        self.tvec = tvec

        projected, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            self.camera_matrix,
            self.dist_coeffs
        )
        projected = projected.reshape(-1, 2)
        error = np.linalg.norm(projected - image_points, axis=1)

        self.get_logger().info('Calibration success.')
        self.get_logger().info(f'Mean reprojection error: {float(np.mean(error)):.3f} px')
        self.get_logger().info(f'Max reprojection error: {float(np.max(error)):.3f} px')
        self.get_logger().info(f'Inliers: {len(inliers) if inliers is not None else 0}/{len(object_points)}')

    def publish_tf(self):
        if self.rvec is None or self.tvec is None:
            return

        rot_mat, _ = cv2.Rodrigues(self.rvec)
        quat = R.from_matrix(rot_mat).as_quat()

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.lidar_frame
        t.child_frame_id = self.camera_frame

        t.transform.translation.x = float(self.tvec[0])
        t.transform.translation.y = float(self.tvec[1])
        t.transform.translation.z = float(self.tvec[2])

        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(t)

    def save_result(self):
        if self.rvec is None or self.tvec is None:
            self.get_logger().warn('No calibration result to save.')
            return

        rot_mat, _ = cv2.Rodrigues(self.rvec)
        quat = R.from_matrix(rot_mat).as_quat()

        data = {
            'parent_frame': self.lidar_frame,
            'child_frame': self.camera_frame,
            'translation': {
                'x': float(self.tvec[0]),
                'y': float(self.tvec[1]),
                'z': float(self.tvec[2]),
            },
            'rotation_quaternion': {
                'x': float(quat[0]),
                'y': float(quat[1]),
                'z': float(quat[2]),
                'w': float(quat[3]),
            },
            'rotation_matrix': rot_mat.tolist(),
            'rvec': self.rvec.reshape(-1).tolist(),
            'tvec': self.tvec.reshape(-1).tolist(),
            'num_pairs': len(self.object_points),
        }

        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)

        with open(self.save_path, 'w') as f:
            yaml.safe_dump(data, f)

        self.get_logger().info(f'Saved calibration result to {self.save_path}')

    def visualize(self):
        if self.latest_image is None:
            return

        img = self.latest_image.copy()

        for i, p in enumerate(self.image_points):
            x, y = int(p[0]), int(p[1])
            cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(img, str(i + 1), (x + 5, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        cv2.putText(
            img,
            f'pairs: {len(self.object_points)} | c:calib s:save d:del q:quit',
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

        cv2.imshow('manual_lidar_camera_calib', img)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):
            self.run_pnp()
        elif key == ord('s'):
            self.save_result()
        elif key == ord('d'):
            if self.object_points:
                self.object_points.pop()
                self.image_points.pop()
                self.get_logger().info('Deleted last pair.')
        elif key == ord('q'):
            rclpy.shutdown()

        self.publish_tf()


def main():
    rclpy.init()
    node = ManualPnPCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()