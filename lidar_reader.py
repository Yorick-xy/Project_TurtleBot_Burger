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

from curses import KEY_B2
import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

K1=10
K2=10
DIST_MIN = 0.4
DIST_MAX = 0.9
DIST_MOY = (DIST_MAX + DIST_MIN) / 2
class MinimalSubscriber(Node):
    
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        
        #self.get_logger().info('I heard: "%s"' % msg.data)
        for i in range(360):
        # #print(msg.ranges[i], msg.intensities[i])
        # #print(msg.ranges[300], msg.angle_max*180/math.pi)
            # print(msg.ranges[i], msg.scan_time)
            sum_x = 0
            sum_y = 0
            nb_valeur =0
            if i>339 or i<10:
                if msg.ranges[i]>DIST_MIN and msg.ranges[i]<DIST_MAX:
                    nb_valeur += 1
                    x = msg.ranges[i] * math.cos(math.radians(i))
                    y = msg.ranges[i] * math.sin(math.radians(i))
                    sum_x = sum_x + x
                    sum_y = sum_y + y
                    print(msg.ranges[i])

        if nb_valeur !=0:
            moyenne_x = sum_x / nb_valeur
            moyenne_y = sum_y / nb_valeur
            
            lin_vel = (moyenne_x + DIST_MOY) * K1
            rot_vel = (moyenne_y - DIST_MOY) * K2
        else:
            lin_vel = 0
            rot_vel = 0

        #TODO Publish lin_vel et rot_vel





    def listener_range(self, msg):
        for i in range (360):
            if (msg.scan_time <= 0.01745329238474369*20 and msg.scan_time >= 0.01745329238474369):
                print(msg.range[i])
            


    


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()