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

import rclpy
import math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from std_msgs.msg import String

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

K1 = 1.5
K2 = 1.0
ANGLE_MAX = 329.0
ANGLE_MIN = 29.0
DIST_MIN = 0.5
DIST_MAX = 1.0
DIST_MOY = (DIST_MAX + DIST_MIN) / 2

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel

class FollowMe(Node):

    def __init__(self):
        self.lin_vel =0.0
        self.rot_vel =0.0
        super().__init__('minimal_publisher')
        qos = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', qos)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        sum_x = 0.0
        sum_y = 0.0
        nb_valeur =0
        for i in range(360):

            if i>ANGLE_MAX or i<ANGLE_MIN:
                if msg.ranges[i]>DIST_MIN and msg.ranges[i]<DIST_MAX:
                    nb_valeur += 1
                    x = msg.ranges[i] * math.cos(math.radians(i))
                    y = msg.ranges[i] * math.sin(math.radians(i))
                    sum_x = sum_x + x
                    sum_y = sum_y + y

        if nb_valeur !=0:
            moyenne_x = sum_x / nb_valeur
            moyenne_y = sum_y / nb_valeur
            self.lin_vel = (moyenne_x - DIST_MOY) * K1
            self.rot_vel = (moyenne_y) * K2

        else:
            self.lin_vel = 0.0
            self.rot_vel = 0.0

        twist = Twist()

        twist.linear.x = self.lin_vel#constrain(self.lin_vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.rot_vel #constrain(self.rot_vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
        print(twist.angular.z)
        
        self.publisher_.publish(twist)
        print("published!")


def main(args=None):
    rclpy.init(args=args)

    follow_me = FollowMe()

    rclpy.spin(follow_me)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    follow_me.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
