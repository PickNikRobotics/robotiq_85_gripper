"""--------------------------------------------------------------------
COPYRIGHT 2014 Stanley Innovation Inc.

Software License Agreement:

The software supplied herewith by Stanley Innovation Inc. (the "Company")
for its licensed Segway RMP Robotic Platforms is intended and supplied to you,
the Company's customer, for use solely and exclusively with Stanley Innovation
products. The software is owned by the Company and/or its supplier, and is
protected under applicable copyright laws.  All rights are reserved. Any use in
violation of the foregoing restrictions may subject the user to criminal
sanctions under applicable laws, as well as to civil liability for the
breach of the terms and conditions of this license. The Company may
immediately terminate this Agreement upon your use of the software with
any products that are not Stanley Innovation products.

The software was written using Python programming language.  Your use
of the software is therefore subject to the terms and conditions of the
OSI- approved open source license viewable at http://www.python.org/.
You are solely responsible for ensuring your compliance with the Python
open source license.

You shall indemnify, defend and hold the Company harmless from any claims,
demands, liabilities or expenses, including reasonable attorneys fees, incurred
by the Company as a result of any claim or proceeding against the Company
arising out of or based upon:

(i) The combination, operation or use of the software by you with any hardware,
    products, programs or data not supplied or approved in writing by the Company,
    if such claim or proceeding would have been avoided but for such combination,
    operation or use.

(ii) The modification of the software by or on behalf of you

(iii) Your use of the software.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

 \file   robotiq_85_driver

 \brief  Node for Robotiq 85 communication

 \Platform: Linux/ROS Foxy
--------------------------------------------------------------------"""

import numpy as np
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from robotiq_85_driver.driver.robotiq_85_gripper import Robotiq85Gripper
from robotiq_85_msgs.msg import GripperCmd, GripperStat
from sensor_msgs.msg import JointState


class Robotiq85Driver(Node):
    def __init__(self):
        super().__init__('robotiq_85_driver')

        self.declare_parameter('num_grippers', 1)
        self.declare_parameter('comport', '/dev/ttyUSB0')
        self.declare_parameter('baud', '115200')

        self._num_grippers = self.get_parameter('num_grippers').get_parameter_value().integer_value
        self._comport = self.get_parameter('comport').get_parameter_value().string_value
        self._baud = self.get_parameter('baud').get_parameter_value().string_value
        
        self.get_logger().info("Parameters Num gippers: %i, Comport: %s, Baud rate: %s " % (self._num_grippers, self._comport, self._baud))

        self._gripper = Robotiq85Gripper(self._num_grippers, self._comport, self._baud)

        if not self._gripper.init_success:
            self.get_logger().error("Unable to open commport to %s: " % self._comport)
            return

        if (self._num_grippers == 1):
            self.create_subscription(GripperCmd, "/gripper/cmd", self._update_gripper_cmd, 10)
            self._gripper_pub = self.create_publisher(GripperStat, '/gripper/stat', 10)
            self._gripper_joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        elif (self._num_grippers == 2):
            self.create_subscription(GripperCmd, "/left_gripper/cmd", self._update_gripper_cmd, 10)
            self._left_gripper_pub = self.create_publisher(GripperStat, '/left_gripper/stat', 10)
            self._left_gripper_joint_state_pub = self.create_publisher(JointState, '/left_gripper/joint_states', 10)
            self.create_subscription(GripperCmd, "/right_gripper/cmd", self._update_right_gripper_cmd, 10)
            self._right_gripper_pub = self.create_publisher(GripperStat, '/right_gripper/stat', 10)
            self._right_gripper_joint_state_pub = self.create_publisher(JointState, '/right_gripper/joint_states', 10)
        else:
            self.get_logger().error("Number of grippers not supported (needs to be 1 or 2)")
            return

        self._prev_js_pos = [0.0] * self._num_grippers
        self._prev_js_time = [self.get_time()] * self._num_grippers
        self._driver_state = 0
        self._driver_ready = False

        success = True
        for i in range(self._num_grippers):
            success &= self._gripper.process_stat_cmd(i)
            if not success:
                bad_gripper = i
        if not success:
            self.get_logger().error("Failed to contact gripper %d....ABORTING"%bad_gripper)
            return

        self._last_time = self.get_time()

        # # 100 Hz timer 
        self.timer = self.create_timer(0.01, self._timer_callback)

    def __del__(self):
        self.shutdown()

    def get_time(self):
        time_msg = self.get_clock().now().to_msg()
        return float(time_msg.sec) + (float(time_msg.nanosec) * 1e-9)


    def shutdown(self):
        self.get_logger().info("Shutdown gripper")
        self._gripper.shutdown()


    def _clamp_cmd(self,cmd,lower,upper):
        if (cmd < lower):
            return lower
        elif (cmd > upper):
            return upper
        else:
            return cmd


    def _update_gripper_cmd(self,cmd):
        if (True == cmd.emergency_release):
            self._gripper.activate_emergency_release(open_gripper=cmd.emergency_release_dir)
            return
        else:
            self._gripper.deactivate_emergency_release()

        if (True == cmd.stop):
            self._gripper.stop()
        else:
            pos = self._clamp_cmd(cmd.position,0.0,0.085)
            vel = self._clamp_cmd(cmd.speed,0.013,0.1)
            force = self._clamp_cmd(cmd.force,5.0,220.0)
            self._gripper.goto(dev=0,pos=pos,vel=vel,force=force)


    def _update_right_gripper_cmd(self,cmd):
        if (True == cmd.emergency_release):
            self._gripper.activate_emergency_release(dev=1,open_gripper=cmd.emergency_release_dir)
            return
        else:
            self._gripper.deactivate_emergency_release(dev=1)

        if (True == cmd.stop):
            self._gripper.stop(dev=1)
        else:
            pos = self._clamp_cmd(cmd.position,0.0,0.085)
            vel = self._clamp_cmd(cmd.speed,0.013,0.1)
            force = self._clamp_cmd(cmd.force,5.0,220.0)
            self._gripper.goto(dev=1,pos=pos,vel=vel,force=force)


    def _update_gripper_stat(self,dev=0):
        stat = GripperStat()
        stat.header.stamp = self.get_clock().now().to_msg()
        stat.is_ready = self._gripper.is_ready(dev)
        stat.is_reset = self._gripper.is_reset(dev)
        stat.is_moving = self._gripper.is_moving(dev)
        stat.obj_detected = self._gripper.object_detected(dev)
        stat.fault_status = self._gripper.get_fault_status(dev)
        stat.position = self._gripper.get_pos(dev)
        stat.requested_position = self._gripper.get_req_pos(dev)
        stat.current = self._gripper.get_current(dev)
        return stat


    def _update_gripper_joint_state(self,dev=0):
        js = JointState()
        js.header.frame_id = ''
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['robotiq_85_left_knuckle_joint']
        pos = np.clip(0.8 - ((0.8/0.085) * self._gripper.get_pos(dev)), 0., 0.8)
        js.position = [pos]
        dt = self.get_time() - self._prev_js_time[dev]
        self._prev_js_time[dev] = self.get_time()
        js.velocity = [(pos-self._prev_js_pos[dev])/dt]
        self._prev_js_pos[dev] = pos
        return js


    def _timer_callback(self):
        dt = self.get_time() - self._last_time
        # print("cb:", dt)

        if (0 == self._driver_state):
            for i in range(self._num_grippers):
                if (dt < 0.5):
                    self._gripper.deactivate_gripper(i)
                else:
                    self._driver_state = 1
        elif (1 == self._driver_state):
            grippers_activated = True
            for i in range(self._num_grippers):
                self._gripper.activate_gripper(i)
                grippers_activated &= self._gripper.is_ready(i)
            if (grippers_activated):
                self._driver_state = 2
        elif (2 == self._driver_state):
            self._driver_ready = True

        for i in range(self._num_grippers):
            success = True
            success &= self._gripper.process_act_cmd(i)
            success &= self._gripper.process_stat_cmd(i)
            if not success:
                self.get_logger().error("Failed to contact gripper %d"%i)
            else:
                stat = GripperStat()
                js = JointState()
                stat = self._update_gripper_stat(i)
                js = self._update_gripper_joint_state(i)
                if (1 == self._num_grippers):
                    self._gripper_pub.publish(stat)
                    self._gripper_joint_state_pub.publish(js)
                else:
                    if (i == 0):
                        self._left_gripper_pub.publish(stat)
                        self._left_gripper_joint_state_pub.publish(js)
                    else:
                        self._right_gripper_pub.publish(stat)
                        self._right_gripper_joint_state_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Robotiq85Driver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()



