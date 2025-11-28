import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import tty
import termios

class ArmJointController(Node):
    def __init__(self):
        super().__init__('arm_joint_controller')

        # Publish to your controller:
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',   # YOUR controller
            10
        )

        # Your 6 joint names from joint_states:
        self.joint_names = [
            'rotating_base',
            'shoulder_joint',
            'elbow_joint',
            'differential_joint',
            'differential_rotation_joint',
            'actuator_joint'
        ]

        # Initial positions:
        self.joint_positions = [0.0] * 6

        # Step size per keypress:
        self.step = 0.1
        self.minus_pressed = False

    def get_key(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def send_trajectory(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.joint_positions

        # 1 second movement duration
        point.time_from_start = rclpy.duration.Duration(seconds=1).to_msg()

        trajectory_msg.points.append(point)
        self.publisher_.publish(trajectory_msg)

        self.get_logger().info(f"Sent: {self.joint_positions}")

    def run(self):
        while rclpy.ok():
            key = self.get_key()

            if key == '-':
                self.minus_pressed = True

            elif key in ['1','2','3','4','5','6']:
                index = int(key) - 1

                if self.minus_pressed:
                    self.joint_positions[index] -= self.step
                    self.minus_pressed = False
                else:
                    self.joint_positions[index] += self.step

                self.send_trajectory()

            elif key == '\x03':  # Ctrl+C
                break


def main(args=None):
    rclpy.init(args=args)
    node = ArmJointController()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
