import re
import subprocess
from random import randint
from math import pi
import time

from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

SPEED = 0.5
CRIT_LEN = 1.5

class MinimalPublisher(Node):

    def __init__(self, name: str):
        super().__init__(name)
        self.name = name
        self.publisher_ = self.create_publisher(Twist, f'model/{self.name}/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            f'{self.name}_lidar',
            self.lidar_callback,
            10)
        self.subscription
        self.lidar_epsilon = 15
        self._logger.info(f"__init__ {self.name}")

    def lidar_callback(self, data):
        #ranges = [round(dist, 3) if dist != float('inf') else -1.0 for dist in data.ranges]
        ranges = []
        for dist in data.ranges:
            if dist != float('inf'):
                ranges.append(round(dist, 3))
            else:
                ranges.append(-1)
        median = len(ranges) // 2
        median_range = ranges[median-self.lidar_epsilon : median+self.lidar_epsilon]
        #self.get_logger().info('MEDIAN: "%s"' % str(ranges[median-self.lidar_epsilon : median+self.lidar_epsilon]))
        s_a = sum(median_range)/len(median_range)
        #self.get_logger().info('----------------------- "%s"' % str(s_a))
        #self.get_logger().info('I heard: "%s"' % str(ranges))
        if s_a >= CRIT_LEN:
            self._move_forward()
        else:
            self._stop()

    def get_coordinates(self) -> dict:
        output = subprocess.Popen(f"ign model -m {self.name} --pose", shell=True, stdout=subprocess.PIPE).stdout.read().decode().replace("\n", "")
        pattern = r'\[(?P<X>-?\d+\.?\d*)\s+(?P<Y>-?\d+\.?\d*)\s+(?P<Z>-?\d+\.?\d*)\]\s+\[(?P<Raw>-?\d+\.?\d*)\s+(?P<Pitch>-?\d+\.?\d*)\s+(?P<Yaw>-?\d+\.?\d*)\]'
        match = re.search(pattern, output)

        return {
            "x": float(match.group("X")),
            "y": float(match.group("Y")),
            "z": float(match.group("Z")),
            "raw": float(match.group("Raw")),
            "pitch": float(match.group("Pitch")),
            "yaw": float(match.group("Yaw")),
        }
    
    def car_run_forward(self):
        self._stop()

    def _move_forward(self):
        msg = Twist()
        msg.linear.x = SPEED
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(msg.linear))

    def _stop(self):
        #time.sleep(1)
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % str(msg.linear) + str(msg.angular))