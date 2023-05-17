# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch.actions import ExecuteProcess

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState


SPEED = 1.0

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'model/vehicle_blue/cmd_vel', 10)
        self._logger.info("__init__")

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
        while True:
            msg = Twist()
            msg.linear.x = 0.5
            msg.angular.z = 0.5
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % str(msg.linear) + str(msg.angular))

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
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(msg.linear) + str(msg.angular))

    def get_coordinates(self):
        self.cli = self.create_client(GetModelState, '/gazebo/get_model_state')
        self.req = GetModelState.Request()
        self.req.model_name = "vehicle_blue"
        self.req.relative_entity_name = "link"
        print(str(self.req))
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        print(self.future.result())
        


        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    minimal_publisher.foo()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
