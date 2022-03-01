"""
*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2021, PickNik Inc
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of PickNik Inc nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************
 Author: Boston Cleek
 File:   single_robotiq_85_action_server
 Brief:  Action server for Robotiq 85 communication
 Platform: Linux/ROS Foxy
"""

import threading
import time
import numpy as np
from math import fabs
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

from robotiq_85_driver.driver.robotiq_85_gripper import Robotiq85Gripper
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from robotiq_85_msgs.msg import GripperCmd, GripperStat


class Robotiq85ActionServer(Node):
    def __init__(self):
        super().__init__('single_robotiq_85_action_server')

        self.declare_parameter('timeout', 5.0)            
        self.declare_parameter('position_tolerance', 0.005) 
        self.declare_parameter('gripper_speed', 0.0565)   

        self._timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self._position_tolerance = self.get_parameter('position_tolerance').get_parameter_value().double_value
        self._gripper_speed = self.get_parameter('gripper_speed').get_parameter_value().double_value

        self.create_subscription(GripperStat, "/gripper/stat", self._update_gripper_stat, 10)
        self._gripper_pub = self.create_publisher(GripperCmd, '/gripper/cmd', 10)

        self._stat = None

        self._cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            GripperCommand,
            '/robotiq_gripper_controller/gripper_cmd',
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            callback_group=self._cb_group)


        self.get_logger().info('Gripper server ready')


    def get_time(self):
        time_msg = self.get_clock().now().to_msg()
        return float(time_msg.sec) + (float(time_msg.nanosec) * 1e-9)


    def shutdown(self):
        self.get_logger().debug("Shutdown gripper")
        self._gripper.shutdown()


    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()


    def _goal_callback(self, goal_request):
        self.get_logger().debug('Gripper received goal request')
        return GoalResponse.ACCEPT


    def _cancel_callback(self, goal_handle):
        self.get_logger().debug('Gripper received cancel request')
        return CancelResponse.ACCEPT


    def _execute_callback(self, goal_handle):
        self.get_logger().debug('Gripper executing goal...')

        # Approximately convert angular joint position at knuckle to linear distance between jaws
        gripper_jaw_distance = min(max(0.085 - (0.085/0.8) * goal_handle.request.command.position, 0.0), 0.8)

        self.get_logger().debug('Angle: ' + str(goal_handle.request.command.position) + ' Distance: ' + str(gripper_jaw_distance))

        # Send goal to gripper 
        cmd_msg = GripperCmd()
        cmd_msg.position = gripper_jaw_distance

        cmd_msg.force = goal_handle.request.command.max_effort
        cmd_msg.speed = self._gripper_speed

        self._gripper_pub.publish(cmd_msg)

        feedback_msg = GripperCommand.Feedback()

        # update at 100Hz
        rate = self.create_rate(100)
        start_time = self.get_time()

        while rclpy.ok():
            dt = self.get_time() - start_time
            # print("cb:", dt)
            
            if not (dt < self._timeout):
                self.get_logger().warn('Gripper timeout reached')
                break


            if self._stat is None:
                self.get_logger().warn("No gripper feedback yet")
                pass
            else:
                feedback_msg.position = self._stat.position
                feedback_msg.stalled = not self._stat.is_moving

                # Position tolerance achieved or object grasped
                if (fabs(gripper_jaw_distance - feedback_msg.position) < self._position_tolerance or self._stat.obj_detected):
                    feedback_msg.reached_goal = True
                    self.get_logger().debug('Goal achieved: %r'% feedback_msg.reached_goal)

                goal_handle.publish_feedback(feedback_msg)

                if feedback_msg.reached_goal:
                    self.get_logger().debug('Reached goal, exiting loop')
                    break;
  
            rate.sleep()

        result_msg = GripperCommand.Result()
        result_msg.reached_goal = feedback_msg.reached_goal
        result_msg.stalled = feedback_msg.stalled
        result_msg.position = feedback_msg.position
        result_msg.effort = feedback_msg.effort

        if result_msg.reached_goal:
            self.get_logger().debug('Setting action to succeeded')
            goal_handle.succeed()
        else:
            self.get_logger().debug('Setting action to abort')
            goal_handle.abort()

        return result_msg


    def _update_gripper_stat(self, data):
        self._stat = data


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Robotiq85ActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
