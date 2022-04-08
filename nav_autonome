# Copyright 2016 Open Source Robotics Foundation, Inc.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from asyncio.base_futures import _FINISHED
import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from nav2_simple_commander import working_on_foxy
from geometry_msgs.msg import PoseStamped

import time
import threading

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

K1=1.5
K2=1.0
ANGLE_MIN = 329
ANGLE_MAX = 29
DIST_MIN = 0.4
DIST_MAX = 1.0
DIST_MOY = (DIST_MAX + DIST_MIN) / 2

class RobotState(working_on_foxy.Enum):
    FOLLOW_ME       = 0
    RETURN_TO_POS   = 1
    FINISHED        = 2

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

        self.current_state = RobotState.RETURN_TO_POS

        #self.navigator.setInitialPose(initial_pose)
    def timer_log(self):
        return (time.time() - self.timer_count)
 
    def timer_milestone(self):
        self.timer_count = time.time()


    def listener_callback(self, msg):
        
        if self.current_state == RobotState.FOLLOW_ME :

            sum_x = 0.0
            sum_y = 0.0
            nb_valeur = 0

            for i in range(360):
                
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
                
                self.rot_vel = moyenne_y * K2
                print("Vitesse rotationnelle : "+str(self.rot_vel))

                MinimalSubscriber.timer_milestone(self)
            else:
                self.lin_vel = 0.0
                self.rot_vel = 0.0

                if(MinimalSubscriber.timer_log(self) >= 3):
                    self.current_state = RobotState.RETURN_TO_POS
                    return

            
            qos = QoSProfile(depth=10)
            node = rclpy.create_node('follow_me')
            pub = node.create_publisher(Twist, 'cmd_vel', qos)

            twist = Twist()
        
            twist.linear.x = constrain(self.lin_vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = constrain(self.rot_vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

            pub.publish(twist)

        elif self.current_state == RobotState.RETURN_TO_POS : #and self.navigator.isNavComplete() :
            navigator = working_on_foxy.BasicNavigatorFoxy()           
            print("Initial point")
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = 8.93
            initial_pose.pose.position.y = -8.84
            initial_pose.pose.orientation.z = 1.0
            initial_pose.pose.orientation.w = 0.0
            navigator.goToPose(initial_pose)
            self.current_state = RobotState.FINISHED

        elif self.current_state == RobotState.FINISHED :
            pass
            
    
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
    minimal_subscriber.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
