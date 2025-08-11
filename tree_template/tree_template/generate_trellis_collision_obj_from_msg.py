#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
from pf_orchard_interfaces.msg import TreeImageData
from tree_template_interfaces.srv import UpdateTrellisPosition
import tf2_ros

class TreeDataListener(Node):
    def __init__(self):
        super().__init__('tree_data_listener')

        self.declare_parameter('camera_delta_topic', '/camera_vertical_delta')
        self.declare_parameter('tree_data_topic', '/tree_image_data')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('tilt_angle_deg', 18.435)

        self.cam_topic    = self.get_parameter('camera_delta_topic').value
        self.tree_topic   = self.get_parameter('tree_data_topic').value
        self.click_topic  = self.get_parameter('clicked_point_topic').value
        self.tilt_deg     = self.get_parameter('tilt_angle_deg').value

        # Service client 
        self.cli = self.create_client(UpdateTrellisPosition, 'update_trellis_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for update_trellis_position service…')

        # State for X/Y/Z anchoring 
        self.tilt_angle   = math.radians(self.tilt_deg)
        self.current_y    = 0.0
        self.anchor_y     = None
        self._last_xyz    = (0.0, 0.0, 0.0)
        self._stored_yaw  = 0.0  # persists between detections

        #  TF buffer for reprojecting clicks 
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

        #  Click buffer 
        self._first_click  = None
        self._second_click = None

        #  Subscriptions 
        self.create_subscription(
            Float32,
            #'/camera_vertical_delta',
            self.cam_topic,
            self.on_vertical_delta,
            10
        )
        self.create_subscription(
            TreeImageData,
            #'/tree_image_data',
            self.tree_topic,
            self.on_tree_data,
            10
        )
        self.create_subscription(
            PointStamped,
            #'/clicked_point',
            self.click_topic,
            self.on_click,
            1
        )

        self.get_logger().info('TreeDataListener initialized and running.')

    def on_vertical_delta(self, msg: Float32):
        # msg.data is total camera movement along camera +Y (down = negative)
        self.current_y = msg.data

    def on_tree_data(self, msg: TreeImageData):
        if not msg.xs:
            self.get_logger().warn('No tree detections')
            return

        # map detection into camera-link X (right), Z (forward)
        x_cam = -msg.xs[0]
        z_cam = -msg.ys[0]

        # anchor at lowest Y seen
        if self.anchor_y is None or self.current_y < self.anchor_y:
            self.anchor_y = self.current_y
        y_cam = self.anchor_y - self.current_y

        # slide base forward along +Z by the tilt angle
        z_offset   = -y_cam * math.tan(self.tilt_angle)
        z_cam_base = z_cam + z_offset

        # stash for click-based yaw updates
        self._last_xyz = (x_cam, y_cam, z_cam_base)

        # publish the template using whichever yaw we have stored
        req = UpdateTrellisPosition.Request()
        req.x   = x_cam
        req.y   = -y_cam      # flip because MoveIt +Y is up
        req.z   = z_cam_base
        req.yaw = self._stored_yaw
        self.cli.call_async(req)

    def on_click(self, msg: PointStamped):
        # transform clicked_point into camera_link frame
        try:
            pt_cam = self.tf_buffer.transform(
                msg,
                'camera_link',
                timeout=Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f"TF error: {e}")
            return

        # buffer the two clicks
        if self._first_click is None:
            self._first_click = pt_cam.point
            self.get_logger().info('First click stored.')
        else:
            self._second_click = pt_cam.point
            self.get_logger().info('Second click stored; computing yaw…')
            self._compute_and_send_yaw()
            self._first_click = None
            self._second_click = None

    def _compute_and_send_yaw(self):
        p1, p2 = self._first_click, self._second_click
        dx = p2.x - p1.x
        dz = p2.z - p1.z

        # yaw about camera Y: + means trunk leans right
        #yaw = math.atan2(dx, dz)
        yaw = math.atan2(dz, dx)


        self._stored_yaw = yaw
        yaw_deg = math.degrees(yaw)
        self.get_logger().info(f"Computed yaw = {yaw_deg:.1f}°")

        # send updated template immediately with new yaw
        x, y, z = self._last_xyz
        req = UpdateTrellisPosition.Request()
        req.x   = x
        req.y   = -y
        req.z   = z
        req.yaw = yaw
        self.cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = TreeDataListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down TreeDataListener…')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
