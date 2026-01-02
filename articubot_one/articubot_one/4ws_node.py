import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import math

class SerialCmdVel(Node):
    def __init__(self):
        super().__init__('serial_cmd_vel')
        
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        time.sleep(2)

        # 🔽 CHANGED TOPIC NAME HERE
        self.sub = self.create_subscription(
            Twist,
            '/diff_cont/cmd_vel_unstamped',
            self.cmd_callback,
            10
        )

        # Rover geometry
        self.L = 0.40   # wheelbase
        self.W = 0.30   # track width
        self.r = 0.065  # wheel radius
        self.EPS = 1e-3

    def to_servo_angle(self, vy, vx):
        delta_deg = math.degrees(math.atan2(vy, vx))   # kinematic angle
        servo = 90.0 + delta_deg                       # shift to servo frame
        return max(0.0, min(180.0, servo))  

    def cmd_callback(self, msg):
        vx = msg.linear.x
        wz = msg.angular.z

        L = self.L
        W = self.W

        x = L / 2
        y = W / 2

        # Front Left
        vfl_x = vx - wz * (+y)
        vfl_y = wz * (+x)

        # Front Right
        vfr_x = vx - wz * (-y)
        vfr_y = wz * (+x)

        # Rear Left
        vrl_x = vx - wz * (+y)
        vrl_y = wz * (-x)

        # Rear Right
        vrr_x = vx - wz * (-y)
        vrr_y = wz * (-x)

        if abs(vx) < self.EPS and abs(wz) < self.EPS:
            d_fl = 90.0
            d_fr = 90.0

        # --------------------------------------------------------
        # General case (vx != 0 OR wz != 0)
        # --------------------------------------------------------
        else:
            # Front-Left wheel velocity
            vfl_x = vx - wz * (+y)
            vfl_y = wz * (+x)

            # Front-Right wheel velocity
            vfr_x = vx - wz * (-y)
            vfr_y = wz * (+x)

            d_fl = self.to_servo_angle(vfl_y, vfl_x)
            d_fr = self.to_servo_angle(vfr_y, vfr_x)

        # Wheel speeds
        v_fl = math.hypot(vfl_x, vfl_y)
        v_fr = math.hypot(vfr_x, vfr_y)
        v_rl = math.hypot(vrl_x, vrl_y)
        v_rr = math.hypot(vrr_x, vrr_y)

        cmd = (
            f"V {v_fl:.2f} {v_fr:.2f} {v_rl:.2f} {v_rr:.2f} "
            f"S {d_fl:.2f} {d_fr:.2f} {d_fl:.2f} {d_fr:.2f}\n"
        )

        self.ser.write(cmd.encode())

def main(args=None):
    rclpy.init(args=args)
    node = SerialCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
