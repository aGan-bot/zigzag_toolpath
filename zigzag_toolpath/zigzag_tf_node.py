import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

import numpy as np
import math

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler

import tf2_ros
import tf2_geometry_msgs


class SurfaceZigZag(Node):

    def __init__(self):
        super().__init__('surface_zigzag')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter("alpha", 1.0)
        self.alpha = float(self.get_parameter("alpha").value)
        self.alpha = max(0.0, min(1.0, self.alpha))

        self.get_logger().info(f"Smoothing alpha: {self.alpha}")

        # -------------------------
        # TF
        # -------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self)

        # -------------------------
        # Joint State
        # -------------------------
        self.current_joint_state = None

        self.create_subscription(
            JointState,
            '/franka/joint_states',
            self.joint_callback,
            10)

        # -------------------------
        # MoveIt Clients
        # -------------------------
        self.cartesian_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path')

        self.execute_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory')

        self.wait_for_system()

        self.run_once()

    # --------------------------------------------------

    def joint_callback(self, msg):
        self.current_joint_state = msg

    # --------------------------------------------------

    def wait_for_system(self):

        self.get_logger().info("Waiting for MoveIt service...")
        self.cartesian_client.wait_for_service()

        self.get_logger().info("Waiting for joint states...")
        while self.current_joint_state is None:
            rclpy.spin_once(self)

        self.get_logger().info("Waiting for TF surface_frame...")
        while rclpy.ok():
            try:
                self.tf_buffer.lookup_transform(
                    "base",
                    "surface_frame",
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0))
                break
            except Exception:
                self.get_logger().warn("Waiting for surface_frame TF...")
                rclpy.spin_once(self)

        self.get_logger().info("System ready.")

    # --------------------------------------------------

    def run_once(self):

        transform = self.tf_buffer.lookup_transform(
            "base",
            "surface_frame",
            rclpy.time.Time())

        size_x = 0.30
        size_y = 0.40
        slice_step_y = 0.04
        tool_offset = 0.02

        waypoints = []
        y_vals = np.arange(-size_y/2, size_y/2, slice_step_y)

        prev_B = 0.0
        prev_C = 0.0

        for row_idx, y in enumerate(y_vals):

            x_vals = np.linspace(-size_x/2, size_x/2, 30)
            if row_idx % 2 == 1:
                x_vals = np.flip(x_vals)

            for x in x_vals:

                # -----------------
                # LOCAL SURFACE FRAME
                # -----------------
                Xn = x / (size_x/2)
                Yn = y / (size_y/2)

                z_surface = 0.05 * (Xn**2 + 0.8*Yn**2)

                dzdx = 0.10 * Xn * (1/(size_x/2))
                dzdy = 0.08 * Yn * (1/(size_y/2))

                A = math.pi
                B_target = -math.atan(dzdx)
                C_target =  math.atan(dzdy)

                B = prev_B + self.alpha * (B_target - prev_B)
                C = prev_C + self.alpha * (C_target - prev_C)

                prev_B = B
                prev_C = C

                q_local = quaternion_from_euler(A, B, C)

                pose_local = PoseStamped()
                pose_local.header.frame_id = "surface_frame"

                pose_local.pose.position.x = x
                pose_local.pose.position.y = y
                pose_local.pose.position.z = z_surface + tool_offset

                pose_local.pose.orientation.x = q_local[0]
                pose_local.pose.orientation.y = q_local[1]
                pose_local.pose.orientation.z = q_local[2]
                pose_local.pose.orientation.w = q_local[3]

                # -----------------
                # TF Transform
                # -----------------
                pose_global = tf2_geometry_msgs.do_transform_pose_stamped(
                    pose_local,
                    transform)

                waypoints.append(pose_global.pose)

        # -------------------------
        # Cartesian Path
        # -------------------------
        req = GetCartesianPath.Request()
        req.group_name = "fr3_arm"
        req.link_name = "fr3_hand_tcp"
        req.waypoints = waypoints
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        req.start_state = RobotState()
        req.start_state.joint_state = self.current_joint_state
        req.start_state.is_diff = False

        future = self.cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        self.get_logger().info(f"Cartesian fraction: {result.fraction}")

        if result.fraction < 0.95:
            self.get_logger().error("Path incomplete. Aborting.")
            return

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = result.solution

        send_goal_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        self.get_logger().info("Execution sent.")


def main():
    rclpy.init()
    node = SurfaceZigZag()
    rclpy.spin_once(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()