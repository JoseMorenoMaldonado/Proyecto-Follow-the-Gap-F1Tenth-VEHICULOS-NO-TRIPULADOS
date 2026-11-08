#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class FollowTheGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap_node')
        
        # ROS 2 Subscriptions & Publishers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        # Algorithmic Tuning Parameters
        self.BUBBLE_RADIUS = 0.6      # Safety bubble radius (meters)
        self.PREPROC_MAX_DIST = 4.5    # Max distance threshold to suppress distant noise
        self.SAFE_DIST_THRESHOLD = 1.2 # Distance threshold for usable space
        
        # Vehicle Speed & Limits
        self.MAX_STEER_ANGLE = 0.4189  # Max steering (~24 degrees)
        self.MIN_SPEED = 1.0           # Cornering speed (m/s)
        self.MAX_SPEED = 10           # Straightaway speed (m/s)
        
        # Lap Timer & Start Line Variables
        self.lap_count = 0
        self.start_time = time.time()
        self.last_lap_time = time.time()
        self.is_near_start = True
        self.START_LINE_RADIUS = 2.0   # Detection radius around start coordinates (meters)
        self.start_x = None
        self.start_y = None

    def odom_callback(self, odom_msg):
        """
        Tracks vehicle position and calculates exact lap times.
        """
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        
        # Register spawn location as official start line
        if self.start_x is None and self.start_y is None:
            self.start_x = x
            self.start_y = y
            self.get_logger().info(f"Start line registered at X: {x:.2f}, Y: {y:.2f}")
            return
            
        dist_to_start = np.hypot(x - self.start_x, y - self.start_y)
        current_time = time.time()
        
        # Detect start line crossing after completing a loop (>10 seconds)
        if dist_to_start < self.START_LINE_RADIUS:
            if not self.is_near_start and (current_time - self.last_lap_time) > 10.0:
                self.lap_count += 1
                lap_duration = current_time - self.last_lap_time
                self.get_logger().info(
                    f"--- LAP {self.lap_count} COMPLETED --- Lap Time: {lap_duration:.2f} s"
                )
                self.last_lap_time = current_time
                self.is_near_start = True
        else:
            self.is_near_start = False

    def preprocess_lidar(self, ranges, angle_min, angle_increment):
        proc_ranges = np.array(ranges)
        
        fov_deg = 90.0
        fov_rad = np.radians(fov_deg)
        start_idx = int((-fov_rad - angle_min) / angle_increment)
        end_idx = int((fov_rad - angle_min) / angle_increment)
        
        start_idx = max(0, start_idx)
        end_idx = min(len(proc_ranges), end_idx)
        
        proc_ranges = proc_ranges[start_idx:end_idx]
        proc_ranges = np.nan_to_num(proc_ranges, nan=0.0, posinf=self.PREPROC_MAX_DIST, neginf=0.0)
        proc_ranges = np.clip(proc_ranges, 0.0, self.PREPROC_MAX_DIST)
        
        kernel_size = 5
        proc_ranges = np.convolve(proc_ranges, np.ones(kernel_size)/kernel_size, mode='same')
        
        return proc_ranges, start_idx

    def find_max_gap(self, free_space_ranges):
        masked = free_space_ranges > self.SAFE_DIST_THRESHOLD
        gap_starts = np.where(np.diff(np.pad(masked.astype(int), (1, 1))) == 1)[0]
        gap_ends = np.where(np.diff(np.pad(masked.astype(int), (1, 1))) == -1)[0]
        
        if len(gap_starts) == 0:
            max_idx = np.argmax(free_space_ranges)
            return max_idx, max_idx
        
        gap_lengths = gap_ends - gap_starts
        best_gap_idx = np.argmax(gap_lengths)
        
        return gap_starts[best_gap_idx], gap_ends[best_gap_idx] - 1

    def find_best_point(self, start_idx, end_idx, ranges):
        gap_ranges = ranges[start_idx:end_idx + 1]
        max_val_idx = np.argmax(gap_ranges)
        center_idx = (start_idx + end_idx) // 2
        best_idx = int(0.6 * (start_idx + max_val_idx) + 0.4 * center_idx)
        return best_idx

    def scan_callback(self, scan_msg):
        proc_ranges, offset_idx = self.preprocess_lidar(
            scan_msg.ranges, scan_msg.angle_min, scan_msg.angle_increment
        )
        
        closest_idx = np.argmin(proc_ranges)
        closest_dist = proc_ranges[closest_idx]
        
        angle_per_index = scan_msg.angle_increment
        if closest_dist > 0:
            bubble_angle_span = np.arctan2(self.BUBBLE_RADIUS, closest_dist)
            bubble_index_span = int(np.ceil(bubble_angle_span / angle_per_index))
        else:
            bubble_index_span = 10
            
        b_start = max(0, closest_idx - bubble_index_span)
        b_end = min(len(proc_ranges), closest_idx + bubble_index_span)
        
        proc_ranges[b_start:b_end] = 0.0
        
        gap_start, gap_end = self.find_max_gap(proc_ranges)
        best_idx_relative = self.find_best_point(gap_start, gap_end, proc_ranges)
        best_idx_global = offset_idx + best_idx_relative
        
        steering_angle = scan_msg.angle_min + (best_idx_global * scan_msg.angle_increment)
        steering_angle = np.clip(steering_angle, -self.MAX_STEER_ANGLE, self.MAX_STEER_ANGLE)
        
        speed = self.MAX_SPEED * (1.0 - (abs(steering_angle) / self.MAX_STEER_ANGLE)**2.2)
        speed = max(self.MIN_SPEED, speed)
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)
        
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
