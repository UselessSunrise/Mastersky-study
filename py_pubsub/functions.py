import re
import subprocess
from random import randint
from math import pi
import time

from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


SPEED = 0.5

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
        self.subscription  # prevent unused variable warning
        self.lidar_epsilon = 15
        self._logger.info(f"__init__ {self.name}")

    def lidar_callback(self, data):
        # ranges = [round(dist, 3) if dist != float('inf') else -1.0 for dist in data.ranges]
        ranges = []
        for dist in data.ranges:
            if dist != float('inf'):
                ranges.append(round(dist, 3))
            else:
                ranges.append(-1.0)
        
        median = len(ranges) // 2
        median_range = ranges[median - self.lidar_epsilon: median + self.lidar_epsilon]
        average = sum(median_range)/len(median_range)
        self.get_logger().info('Median range: "%s"\nAverage dist: "%s"' % (str(median_range), str(round(average, 3))))
        # self.get_logger().info('I heard: "%s"' % str(ranges))
        
    def listen(self):
        self._logger.info("listen")
        while True:
            move = str(input())
            self._logger.info(f"move: {move}")
            match move:
                case 'w':
                    self._move_forward()
                case 's':
                    self._move_back()
                case 'a':
                    self._turn_left()
                case 'd':
                    self._turn_right()
                case ' ':
                    self._stop()
        
    def foo(self):
        target_coordinates = {"x": randint(-5.0, 5.0), "y": randint(-5.0, 5.0)}
        target_coordinates = {"x": -1.0, "y": -1.0}
        self._logger.info(f"Target coordinates: {target_coordinates}")
        start_coordinates = self.get_coordinates()

        current_coordinates = self.get_coordinates()

        if start_coordinates['x'] > target_coordinates['x']:
            while abs(current_coordinates["yaw"] - start_coordinates["yaw"]) < pi:
                self._logger.info(f"Current: {current_coordinates['yaw']}")
                self._logger.info(f"Start: {start_coordinates['yaw']}")
                msg = Twist()
                msg.angular.z = SPEED/2
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing: "%s"' % str(msg.angular))
                current_coordinates = self.get_coordinates()

        while abs(current_coordinates["x"] - start_coordinates["x"]) < target_coordinates["x"]:
            msg = Twist()
            msg.linear.x = SPEED
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % str(msg.linear) + "\nMy current x coordinate: " + str(current_coordinates["x"]))
            current_coordinates["x"] = self.get_coordinates()['x']

        if start_coordinates['y'] < target_coordinates['y']:
            while abs(current_coordinates["yaw"] - start_coordinates["yaw"]) < pi/2:
                msg = Twist()
                msg.angular.z = -SPEED/2
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing: "%s"' % str(msg.angular))
                current_coordinates = self.get_coordinates()
        else:
            while abs(current_coordinates["yaw"] - start_coordinates["yaw"]) < pi/2:
                msg = Twist()
                msg.angular.z = SPEED/2
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing: "%s"' % str(msg.angular))
                current_coordinates = self.get_coordinates()

        current_coordinates = self.get_coordinates()
        while abs(current_coordinates["y"] - start_coordinates["y"]) < target_coordinates["y"]:
            msg = Twist()
            msg.linear.x = SPEED
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % str(msg.linear) + "\nMy current y coordinate: " + str(current_coordinates["y"]))
            current_coordinates["y"] = self.get_coordinates()['y']
        
        self._stop()
        self.get_logger().info(f"My current coordinates: {current_coordinates}")

    def _move_forward(self):
        msg = Twist()
        msg.linear.x = SPEED
        msg.linear.z = SPEED
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(msg.linear))

    def _turn_right(self):
        msg = Twist()
        msg.angular.z = -SPEED
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(msg.angular))

    def _turn_left(self):
        msg = Twist()
        msg.angular.z = SPEED
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(msg.angular))

    def _move_back(self):
        msg = Twist()
        msg.linear.x = -SPEED
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(msg.linear))

    def _stop(self):
        time.sleep(1)
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(msg.linear) + str(msg.angular))

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