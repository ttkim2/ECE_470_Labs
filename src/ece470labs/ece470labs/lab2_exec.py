#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates
import time
import numpy as np
from math import pi
import sys
class JointAngles:
    def __init__(self):
        self.name = ["", "", "", "", "", ""]  #could have also done [""] * 6
        self.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# UR3e home position
home = np.radians([185, -63, 76, -105, -88, 2])

# Hanoi tower location 
Q00 = [175.53*pi/180.0, -62.73*pi/180.0, 99.25*pi/180.0, -123.50*pi/180.0, -90.12*pi/180.0, 0.0*pi/180.0]
Q01 = [176.13*pi/180.0, -57.62*pi/180.0, 101.64*pi/180.0, -131.02*pi/180.0, -90.92*pi/180.0, 0.87*pi/180.0]
Q02 = [176.45*pi/180.0, -51.94*pi/180.0, 103.30*pi/180.0, -138.03*pi/180.0, -91.07*pi/180.0, 0.87*pi/180.0]
Q10 = [191.55*pi/180.0, -33.32*pi/180.0, 47.19*pi/180.0, -105.78*pi/180.0, -90.23*pi/180.0, 0.87*pi/180.0]
Q11 = [191.56*pi/180.0, -29.43*pi/180.0, 47.21*pi/180.0, -106.95*pi/180.0, -90.32*pi/180.0, 0.87*pi/180.0]
Q12 = [191.85*pi/180.0, -25.18*pi/180.0, 47.21*pi/180.0, -110.46*pi/180.0, -91.68*pi/180.0, 0.99*pi/180.0]
Q20 = [197.92*pi/180.0, -53.94*pi/180.0, 82.55*pi/180.0, -113.43*pi/180.0, -93.25*pi/180.0, 0.92*pi/180.0]
Q21 = [197.88*pi/180.0, -50.95*pi/180.0, 88.83*pi/180.0, -125.73*pi/180.0, -93.26*pi/180.0, 0.95*pi/180.0]
Q22 = [198.39*pi/180.0, -45.60*pi/180.0, 89.74*pi/180.0, -131.90*pi/180.0, -93.27*pi/180.0, 0.96*pi/180.0]
"""
Initialize Q matrix
"""

Q = [ [Q00, Q01, Q02], \
      [Q10, Q11, Q12], \
      [Q20, Q21, Q22] ]
############### Your Code End Here ###############
class UR3e(Node):
    def __init__(self):
        super().__init__('ur3e')

        # Publishers
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        ############## Your Code Start Here ##############
        # TODO: define a ROS subscriber for gripper input message and corresponding callback function
        # ROS2 gripper input topic: /io_and_status_controller/io_states
        self.io_state_sub = self.create_subscription(IOStates, '/io_and_status_controller/io_states', self.io_state_callback, 10)

        ############### Your Code End Here ###############

        # Service clients
        self.io_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not self.io_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('IO service not available, waiting...')

        # State variables
        self.current_joint_state = None
        self.analog_in_0_value = 0
        self.current_JointAngles = JointAngles()
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ] # shoulder_pan_joint is the base rotation joint

    def joint_state_callback(self, msg):
        self.current_joint_state = msg  # Currently only used to check if messages have arrived
        index_inOrder = 0
        for name in self.joint_names:
            index_outofOrder = msg.name.index(name)
            self.current_JointAngles.name[index_inOrder] = name
            self.current_JointAngles.position[index_inOrder] = msg.position[index_outofOrder]
            index_inOrder = index_inOrder + 1 


    def io_state_callback(self, msg):
    ############## Your Code Start Here ##############
        """
        TODO: define a ROS topic callback funtion that 
        receives and stores the state of  the suction cup
        Whenever /io_and_status_controller/io_states 
        publishes this info, this callback function is
        called.
        """
        if len(msg.analog_in_states) > 0:
            self.analog_in_0_value = msg.analog_in_states[0].state

    ############### Your Code End Here ###############

    def set_io(self, pin, state):
        req = SetIO.Request()
        req.fun = 1
        req.pin = pin
        req.state = state
        future = self.io_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


    def move_arm(self, target):
        if self.current_joint_state is None:
            self.get_logger().error("No joint state received!")
            return False

        V_MAX = 1#2.09    # rad/s
        A_MAX = 0.8#2.79   # rad/s^2
        MIN_DURATION = 1
        MAX_DURATION = 8.0

        deltas = []
        for i in range(6):
            deltas.append(abs(self.current_JointAngles.position[i] - target[i]))


        max_delta = max(deltas)
        t_acc = V_MAX / A_MAX
        d_acc = 0.5 * A_MAX * (t_acc ** 2)
        if max_delta > 2 * d_acc:
            # trapezoidal velocity profile
            t_total = 2 * t_acc + (max_delta - 2 * d_acc) / V_MAX
        else:
            # triangular velocity profile
            t_total = 2 * (max_delta / A_MAX) ** 0.5

        duration = max(MIN_DURATION, min(t_total, MAX_DURATION))

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # Start immediately when the controller receives it
        trajectory_msg.header.stamp.sec = 0
        trajectory_msg.header.stamp.nanosec = 0

        # Anchor point: current measured joint state at t = 0
        p0 = JointTrajectoryPoint()
        p0.positions = self.current_JointAngles.position
        p0.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # starting at rest
        p0.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # starting at rest
        p0.time_from_start.sec = 0
        p0.time_from_start.nanosec = 0
        trajectory_msg.points.append(p0)

        # Goal point
        p1 = JointTrajectoryPoint()
        p1.positions = target
        p1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # end at rest, 2 point trajectory
        p1.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #end at rest.
        p1.time_from_start.sec = int(duration)
        p1.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        trajectory_msg.points.append(p1)

        self.trajectory_pub.publish(trajectory_msg)

        self.get_logger().info(f'Moving to position: {np.degrees(target)}')

        # Wait for movement completion
        start_time = time.time()
        while time.time() - start_time < duration + 2:
            rclpy.spin_once(self, timeout_sec=0.1)

            deltas = []
            for i in range(6):
                deltas.append(abs(self.current_JointAngles.position[i] - target[i]))
            if all(delta < 0.001 for delta in deltas):
                time.sleep(0.25)
                return True
        return False



    def move_block(self, start_tower, start_height, end_tower, end_height):
        global Q
    ############## Your Code Start Here ##############
    # TODO: add code to move block from start tower and height to end tower and height
    ### Hint: Use the Q array to map out your towers by location and "height".

        error = 0
        self.move_arm(home)
        self.move_arm(Q[start_tower][start_height])
        time.sleep(0.5)
        self.set_io(0, 1.0)  # Turn on suction

        start_time = time.time()
        while time.time() - start_time < 1:
            rclpy.spin_once(self, timeout_sec=0.1)

        if (self.analog_in_0_value < 2):
            error = 1
            print("Error: Suction cup did not pick up the block.")
            self.set_io(0, 0.0)  # Turn off suction
            sys.exit()

        time.sleep(0.5)
        self.move_arm(home)
        self.move_arm(Q[end_tower][end_height])
        time.sleep(0.5)
        self.set_io(0, 0.0)  # Turn off suction
        time.sleep(0.5)
        return error

    ############### Your Code End Here ###############


def main(args=None):
    input("Check if the UR3e is in 'Remote' Mode?\n\
    Check if the UR3e is initialized and in 'Normal' state.\n\
    Have you run the ROS2 launch statement?\n\
    If there was an UR3e emergency stop or error, Ctrl-C the ros2 launch and rerun.\n\
    \n\
    Press <Enter> to Continue.")
    rclpy.init(args=args)
    node = UR3e()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input
    loop_count = 0
    # Wait for initial state updates
    while node.current_joint_state is None:
        executor.spin_once(timeout_sec=0.05)
        node.get_logger().info("Waiting for initial state updates...")
        time.sleep(0.5)

    try:
        # Get user input
        input_string_start = input("Enter Start Tower (1, 2, 3) or 0 to quit:")
        input_string_end = input("Enter End Tower (1, 2, 3) or 0 to quit:")
        print("You entered " + input_string_start + " and " + input_string_end + "\n")

        if (int(input_string_start) == int(input_string_end)):
            print("Start and End Towers cannot be the same. Please try again.\n\n")
            sys.exit()
        
        if(int(input_string_start) == 1):
            start = 0
        elif (int(input_string_start) == 2):
            start = 1
        elif (int(input_string_start) == 3):
            start = 2
        elif (int(input_string_start) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")

        if(int(input_string_end) == 1):
            end = 0
        elif (int(input_string_end) == 2):
            end = 1
        elif (int(input_string_end) == 3):
            end = 2
        elif (int(input_string_end) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")

        middle = 3 - int(start) - int(end)  # The middle tower is the one not chosen

        ############## Your Code Start Here ##############
        # TODO: modify the code so that UR3e can move tower accordingly from user input
        node.move_block(start, 0, end, 2)
        node.move_block(start, 1, middle, 2)
        node.move_block(end, 2, middle, 1)
        node.move_block(start, 2, end, 2)
        node.move_block(middle, 1, start, 2)
        node.move_block(middle, 2, end, 1)
        node.move_block(start, 2, end, 0)
        node.move_arm(home)
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
