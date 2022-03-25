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
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

K1=0.5
K2=0.3
ANGLE_MIN = 329
ANGLE_MAX = 29
DIST_MIN = 0.4
DIST_MAX = 1
DIST_MOY = (DIST_MAX + DIST_MIN) / 2


class MinimalSubscriber(Node):
    
    def __init__(self):
        self.lin_vel =0.0
        self.rot_vel =0.0
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
            sum_x = 0.0
            sum_y = 0.0
            nb_valeur =0
            if i>ANGLE_MIN or i<ANGLE_MAX:
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
            
            self.lin_vel = (moyenne_x - DIST_MOY) * K1
            print("Vitesse lineaire : "+str(self.lin_vel))
            #if self.rot_vel 
            self.rot_vel = (moyenne_y + DIST_MOY) * K2
            print("Vitesse rotationnelle : "+str(self.rot_vel))
        else:
            self.lin_vel = 0.0
            self.rot_vel = 0.0

        
        qos = QoSProfile(depth=10)
        node = rclpy.create_node('follow_me')
        pub = node.create_publisher(Twist, 'cmd_vel', qos)

        twist = Twist()

        print("test publisher************************************")        
        twist.linear.x = constrain(self.lin_vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = constrain(self.rot_vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

        pub.publish(twist)


        #TODO Publish lin_vel et rot_vel
            
    
def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def main():
    rclpy.init()

    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    # while(1):

    #     rclpy.spin_once(minimal_subscriber)

    #     qos = QoSProfile(depth=10)
    #     node = rclpy.create_node('follow_me')
    #     pub = node.create_publisher(Twist, 'cmd_vel', qos_profile_sensor_data)

    #     twist = Twist()

    #     print("test publisher************************************")        
    #     twist.linear.x = constrain(minimal_subscriber.lin_vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    #     twist.linear.y = 0.0
    #     twist.linear.z = 0.0

    #     twist.angular.x = 0.0
    #     twist.angular.y = 0.0
    #     twist.angular.z = constrain(minimal_subscriber.rot_vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    #     pub.publish(twist)
    # minimal_subscriber.destroy_node()
    # try:
    #     print("test publisher")
    #     while(1):

    #         twist = Twist()
            
    #         twist.linear.x = constrain(minimal_subscriber.lin_vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    #         twist.linear.y = 0.0
    #         twist.linear.z = 0.0

    #         twist.angular.x = 0.0
    #         twist.angular.y = 0.0
    #         twist.angular.z = 0.1 #constrain(minimal_subscriber.rot_vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    #         pub.publish(twist)

    # except Exception as e:
    #     print(e)

    # finally:
    #     twist = Twist()
    #     twist.linear.x = 0.0
    #     twist.linear.y = 0.0
    #     twist.linear.z = 0.0

    #     twist.angular.x = 0.0
    #     twist.angular.y = 0.0
    #     twist.angular.z = 0.0

    #     pub.publish(twist)

    
    rclpy.shutdown()


if __name__ == '__main__':
    main()