import rclpy
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64


class ThrustersControl(Node):

    def __init__(self):
        super().__init__('thrusters_control')
        cos_30 = np.cos(np.pi/6)
        sin_30 = np.sin(np.pi/6)
        self.thrusters_setpoint_ = np.zeros(6)
        self.thruster_control_matrix_ = np.array([[0, 0, cos_30, cos_30, -cos_30, -cos_30],
                                                  [0, 0, -sin_30, sin_30, -sin_30, sin_30],
                                                  [1, 1,       0,      0,       0,      0],
                                                  [0.1932, -0.1932, 0, 0, 0, 0],
                                                  [-0.0277, -0.0277, 0, 0, 0, 0],
                                                  [0, 0, -0.2188, 0.2188, 0.2519, -0.2519]])
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        self.publishers_ = [self.create_publisher(Float64, '/model/brov/joint/propeller_DW_joint/cmd_pos', 10),
                            self.create_publisher(Float64, '/model/brov/joint/propeller_DE_joint/cmd_pos', 10),
                            self.create_publisher(Float64, '/model/brov/joint/propeller_NW_joint/cmd_pos', 10),
                            self.create_publisher(Float64, '/model/brov/joint/propeller_NE_joint/cmd_pos', 10),
                            self.create_publisher(Float64, '/model/brov/joint/propeller_SW_joint/cmd_pos', 10),
                            self.create_publisher(Float64, '/model/brov/joint/propeller_SE_joint/cmd_pos', 10)]
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float64()
        for i, pub in enumerate(self.publishers_):
            msg.data = float(self.thrusters_setpoint_[i])
            pub.publish(msg)

    def listener_callback(self, msg):
        effort_setpoints = np.array([[msg.linear.x],
                                     [msg.linear.y],
                                     [msg.linear.z],
                                     [msg.angular.x],
                                     [msg.angular.y],
                                     [msg.angular.z]])
        print(self.thruster_control_matrix_.T)
        print(effort_setpoints)
        self.thrusters_setpoint_ = self.thruster_control_matrix_.T @ (effort_setpoints)


def main(args=None):
    rclpy.init(args=args)

    thrusters_control = ThrustersControl()

    rclpy.spin(thrusters_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    thrusters_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()