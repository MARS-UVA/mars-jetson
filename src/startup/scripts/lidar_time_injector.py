#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

class LidarPreprocessor(Node):
    def __init__(self):
        super().__init__('lidar_preprocessor')

        self.sub = self.create_subscription(
            PointCloud2,
            '/unilidar/cloud_raw',
            self.cloud_callback,
            10
        )
        self.pub = self.create_publisher(PointCloud2, '/unilidar/cloud', 10)

        # URDF / Sensor parameters
        self.num_rings = 32
        self.min_elevation = -0.261799  # -15 deg
        self.max_elevation =  0.261799  # +15 deg
        self.scan_period = 0.1          # 10 Hz
        self.elev_range = self.max_elevation - self.min_elevation

        # 22-byte packed layout: x(4), y(4), z(4), intensity(4), ring(2), time(4)
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
            PointField(name='time', offset=18, datatype=PointField.FLOAT32, count=1),
        ]

        # Explicitly declare matching structured dtype
        self.cloud_dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
            ('ring', np.uint16),
            ('time', np.float32)
        ], align=False)  # align=False ensures packed 22 bytes with time at offset 18

    def cloud_callback(self, msg: PointCloud2):
        # 1. Read input points
        pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z', 'intensity'))
        if pts.shape[0] == 0:
            return

        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]
        intensity = pts[:, 3]

        # 2. Vectorized range & finiteness filtering
        r2 = x*x + y*y + z*z
        valid_mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (r2 >= 0.01) & (r2 <= 900.0)

        x = x[valid_mask]
        y = y[valid_mask]
        z = z[valid_mask]
        intensity = intensity[valid_mask]
        r = np.sqrt(r2[valid_mask])

        if len(x) == 0:
            return

        # 3. Vectorized relative time calculation [0.0, scan_period]
        yaw = np.arctan2(y, x)
        yaw = np.where(yaw < 0.0, yaw + (2.0 * np.pi), yaw)
        t_offset = ((yaw / (2.0 * np.pi)) * self.scan_period).astype(np.float32)

        # 4. Vectorized ring assignment
        sin_elev = np.clip(z / r, -1.0, 1.0)
        elevation = np.arcsin(sin_elev)
        norm_elev = (elevation - self.min_elevation) / self.elev_range
        ring = np.clip(np.round(norm_elev * (self.num_rings - 1)), 0, self.num_rings - 1).astype(np.uint16)

        # 5. Populate structured NumPy array with matching dtype
        structured_pts = np.empty(len(x), dtype=self.cloud_dtype)
        structured_pts['x'] = x
        structured_pts['y'] = y
        structured_pts['z'] = z
        structured_pts['intensity'] = intensity
        structured_pts['ring'] = ring
        structured_pts['time'] = t_offset

        # 6. Publish (explicitly set point_step=22)
        out_msg = pc2.create_cloud(msg.header, self.fields, structured_pts, point_step=22)
        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarPreprocessor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
