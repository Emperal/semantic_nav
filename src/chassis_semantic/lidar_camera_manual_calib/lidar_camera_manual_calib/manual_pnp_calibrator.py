import os
import yaml
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped
from cv_bridge import CvBridge

from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from scipy.spatial.transform import Rotation as R


def transform_msg_to_matrix(msg: TransformStamped):
    q = msg.transform.rotation
    t = msg.transform.translation

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def matrix_to_xyz_quat(T):
    xyz = T[:3, 3]
    quat = R.from_matrix(T[:3, :3]).as_quat()  # x y z w
    return xyz, quat


class ManualPnPCalibrator(Node):
    def __init__(self):
        super().__init__('manual_pnp_calibrator')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('clicked_point_topic', '/clicked_point')

        self.declare_parameter('lidar_frame', 'livox_frame')
        self.declare_parameter('camera_link_frame', 'camera_link')
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')

        self.declare_parameter('save_path', '/tmp/lidar_camera_extrinsic.yaml')

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.clicked_point_topic = self.get_parameter('clicked_point_topic').value

        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.camera_link_frame = self.get_parameter('camera_link_frame').value
        self.camera_optical_frame = self.get_parameter('camera_optical_frame').value

        self.save_path = self.get_parameter('save_path').value

        self.bridge = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.camera_matrix = None
        self.dist_coeffs = None

        self.latest_image = None
        self.latest_clicked_3d = None

        self.object_points = []
        self.image_points = []

        # PnP result:
        # T_camera_optical_lidar: lidar point -> camera optical point
        self.rvec = None
        self.tvec = None
        self.T_camera_optical_lidar = None

        # Final TF result:
        # T_lidar_camera_link: camera_link pose under lidar frame
        self.T_lidar_camera_link = None

        self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
        self.create_subscription(PointStamped, self.clicked_point_topic, self.clicked_point_callback, 10)

        self.timer = self.create_timer(0.03, self.visualize)

        cv2.namedWindow('manual_lidar_camera_calib')
        cv2.setMouseCallback('manual_lidar_camera_calib', self.mouse_callback)

        self.get_logger().info('Manual PnP calibrator started.')
        self.get_logger().info('RViz Publish Point first, then click same point in image.')
        self.get_logger().info('Press c: calibrate, s: save, d: delete last pair, q: quit.')
        self.get_logger().info(f'PnP target frame: {self.camera_optical_frame}')
        self.get_logger().info(f'Final TF will be: {self.lidar_frame} -> {self.camera_link_frame}')

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)

            self.get_logger().info(
                f'Got camera info from {self.camera_info_topic}'
            )
            self.get_logger().info(f'K={self.camera_matrix.tolist()}')
            self.get_logger().info(f'D={self.dist_coeffs.tolist()}')

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')

    def clicked_point_callback(self, msg):
        if msg.header.frame_id != self.lidar_frame:
            self.get_logger().warn(
                f'Clicked point frame is {msg.header.frame_id}, '
                f'but expected {self.lidar_frame}. '
                f'Please set RViz Fixed Frame to {self.lidar_frame}.'
            )

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

    def get_camera_link_to_optical_tf(self):
        """
        获取 Orbbec 已经发布的:
            camera_link -> camera_color_optical_frame

        tf2 lookup_transform(target, source) 返回的是:
            source -> target

        所以：
            lookup_transform(camera_link, camera_optical)
        得到的是:
            optical -> camera_link 的变换矩阵 T_camera_link_camera_optical

        但我们需要 parent camera_link child optical 的矩阵:
            T_camera_link_camera_optical
        正好可以直接使用。
        """
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.camera_link_frame,
                self.camera_optical_frame,
                rclpy.time.Time()
            )
            return transform_msg_to_matrix(tf_msg)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(
                f'Cannot get TF {self.camera_link_frame} <- {self.camera_optical_frame}: {e}'
            )
            return None

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

        rot_mat, _ = cv2.Rodrigues(self.rvec)

        self.T_camera_optical_lidar = np.eye(4, dtype=np.float64)
        self.T_camera_optical_lidar[:3, :3] = rot_mat
        self.T_camera_optical_lidar[:3, 3] = self.tvec.reshape(3)

        projected, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            self.camera_matrix,
            self.dist_coeffs
        )

        projected = projected.reshape(-1, 2)
        error = np.linalg.norm(projected - image_points, axis=1)

        self.get_logger().info('PnP calibration success.')
        self.get_logger().info(f'Mean reprojection error: {float(np.mean(error)):.3f} px')
        self.get_logger().info(f'Max reprojection error: {float(np.max(error)):.3f} px')
        self.get_logger().info(f'Inliers: {len(inliers) if inliers is not None else 0}/{len(object_points)}')

        self.compute_final_lidar_to_camera_link_tf()


    def compute_final_lidar_to_camera_link_tf(self):
        """
        PnP 得到的是:
            T_camera_optical_lidar

        Orbbec 已有:
            T_camera_link_camera_optical

        最终要发布:
            T_lidar_camera_link

        推导:
            T_lidar_camera_optical = inv(T_camera_optical_lidar)

            T_camera_link_camera_optical 是 optical 在 camera_link 下的位姿
            所以:
            T_camera_optical_camera_link = inv(T_camera_link_camera_optical)

            T_lidar_camera_link =
                T_lidar_camera_optical * T_camera_optical_camera_link
        """
        if self.T_camera_optical_lidar is None:
            return

        T_camera_link_camera_optical = self.get_camera_link_to_optical_tf()

        if T_camera_link_camera_optical is None:
            self.get_logger().error(
                'Cannot compute final lidar -> camera_link TF because camera_link -> optical TF is missing.'
            )
            return

        T_lidar_camera_optical = np.linalg.inv(self.T_camera_optical_lidar)
        T_camera_optical_camera_link = np.linalg.inv(T_camera_link_camera_optical)

        self.T_lidar_camera_link = (
            T_lidar_camera_optical @ T_camera_optical_camera_link
        )

        xyz, quat = matrix_to_xyz_quat(self.T_lidar_camera_link)

        self.get_logger().info('Final TF computed:')
        self.get_logger().info(
            f'{self.lidar_frame} -> {self.camera_link_frame}'
        )
        self.get_logger().info(
            f'translation xyz = {xyz.tolist()}'
        )
        self.get_logger().info(
            f'quaternion xyzw = {quat.tolist()}'
        )
        self.get_logger().info(
            'static_transform_publisher command:'
        )
        self.get_logger().info(
            f'ros2 run tf2_ros static_transform_publisher '
            f'{xyz[0]} {xyz[1]} {xyz[2]} '
            f'{quat[0]} {quat[1]} {quat[2]} {quat[3]} '
            f'{self.lidar_frame} {self.camera_link_frame}'
        )

    def publish_tf(self):
        if self.T_lidar_camera_link is None:
            return

        xyz, quat = matrix_to_xyz_quat(self.T_lidar_camera_link)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.lidar_frame
        t.child_frame_id = self.camera_link_frame

        t.transform.translation.x = float(xyz[0])
        t.transform.translation.y = float(xyz[1])
        t.transform.translation.z = float(xyz[2])

        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(t)

    def save_result(self):
        if self.T_camera_optical_lidar is None:
            self.get_logger().warn('No PnP result to save.')
            return

        if self.T_lidar_camera_link is None:
            self.compute_final_lidar_to_camera_link_tf()

        if self.T_lidar_camera_link is None:
            self.get_logger().warn('No final lidar -> camera_link result to save.')
            return

        xyz_link, quat_link = matrix_to_xyz_quat(self.T_lidar_camera_link)
        xyz_optical_lidar, quat_optical_lidar = matrix_to_xyz_quat(self.T_camera_optical_lidar)

        data = {
            'description': 'LiDAR-camera calibration result. Final TF is lidar_frame -> camera_link_frame.',
            'frames': {
                'lidar_frame': self.lidar_frame,
                'camera_link_frame': self.camera_link_frame,
                'camera_optical_frame': self.camera_optical_frame,
            },

            'final_tf_lidar_to_camera_link': {
                'parent_frame': self.lidar_frame,
                'child_frame': self.camera_link_frame,
                'translation': {
                    'x': float(xyz_link[0]),
                    'y': float(xyz_link[1]),
                    'z': float(xyz_link[2]),
                },
                'rotation_quaternion_xyzw': {
                    'x': float(quat_link[0]),
                    'y': float(quat_link[1]),
                    'z': float(quat_link[2]),
                    'w': float(quat_link[3]),
                },
                'matrix': self.T_lidar_camera_link.tolist(),
            },

            'pnp_result_lidar_to_camera_optical_math': {
                'note': 'This is the OpenCV solvePnP transform: point_in_camera_optical = R * point_in_lidar + t.',
                'target_frame': self.camera_optical_frame,
                'source_frame': self.lidar_frame,
                'translation': {
                    'x': float(self.tvec[0]),
                    'y': float(self.tvec[1]),
                    'z': float(self.tvec[2]),
                },
                'rotation_quaternion_xyzw': {
                    'x': float(quat_optical_lidar[0]),
                    'y': float(quat_optical_lidar[1]),
                    'z': float(quat_optical_lidar[2]),
                    'w': float(quat_optical_lidar[3]),
                },
                'matrix': self.T_camera_optical_lidar.tolist(),
                'rvec': self.rvec.reshape(-1).tolist(),
                'tvec': self.tvec.reshape(-1).tolist(),
            },

            'num_pairs': len(self.object_points),
            'object_points_lidar': [p.tolist() for p in self.object_points],
            'image_points_pixel': [p.tolist() for p in self.image_points],
        }

        save_dir = os.path.dirname(self.save_path)
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)

        with open(self.save_path, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)

        self.get_logger().info(f'Saved calibration result to {self.save_path}')

    def visualize(self):
        if self.latest_image is None:
            return

        img = self.latest_image.copy()

        for i, p in enumerate(self.image_points):
            x, y = int(p[0]), int(p[1])
            cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(
                img,
                str(i + 1),
                (x + 5, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1
            )

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