import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_matrix
import numpy as np

class CartesianZigZagDebug(Node):

    def __init__(self):
        super().__init__('cartesian_zigzag_debug')

        # Robot base link offset
        self.robot_origin_offset = np.array([0.35, 0.0, 0.35])  # meters

        # Parametreler
        self.size_x = 0.3   # meters
        self.size_y = 0.4
        self.slice_step_y = 0.04
        self.points_per_row = 5
        self.offset_dist = 0.02
        self.base_z = self.robot_origin_offset[2]

        # Waypointleri oluştur
        poses_6d = self.generate_toolpath()

        # Debug: terminale yazdır
        self.print_waypoints(poses_6d)

    # -----------------------------
    # Toolpath üretimi (stable TCP frame)
    # -----------------------------
    def generate_toolpath(self):
        x_dense = np.linspace(-self.size_x/2, self.size_x/2, 200)
        y_vals = np.arange(-self.size_y/2, self.size_y/2, self.slice_step_y)
        X_dense, Y_dense = np.meshgrid(x_dense, y_vals)

        # Normalizasyon ve Z hesaplama (base_z + ~10 cm)
        X_norm = X_dense / (self.size_x/2)
        Y_norm = Y_dense / (self.size_y/2)
        Z_dense = 0.1 * (X_norm**2 + 0.8*Y_norm**2) + self.base_z

        # Normal hesaplama
        dx = x_dense[1] - x_dense[0]
        dy = self.slice_step_y
        dz_dx = np.gradient(Z_dense, axis=1)/dx
        dz_dy = np.gradient(Z_dense, axis=0)/dy
        nx = -dz_dx
        ny = -dz_dy
        nz = np.ones_like(Z_dense)
        norm = np.sqrt(nx**2 + ny**2 + nz**2)
        nx /= norm
        ny /= norm
        nz /= norm

        # Toolpath + normal listesi
        tool_positions = []
        normals_list = []
        for i in range(len(y_vals)):
            indices = np.linspace(0, len(x_dense)-1, self.points_per_row).astype(int)
            if i % 2 == 1:
                indices = indices[::-1]

            for j in indices:
                px = X_dense[i, j]
                py = Y_dense[i, j]
                pz = Z_dense[i, j]
                normal = np.array([nx[i,j], ny[i,j], nz[i,j]])
                normal /= np.linalg.norm(normal)
                offset_point = np.array([px, py, pz]) + self.offset_dist * normal

                # Base link offset
                offset_point += self.robot_origin_offset

                tool_positions.append(offset_point)
                normals_list.append(normal)

        tool_positions = np.array(tool_positions)
        normals_list = np.array(normals_list)

        # Stable TCP frame
        frames = []
        global_x = np.array([1.0, 0.0, 0.0])
        for k in range(len(tool_positions)):
            p = tool_positions[k]
            z_axis = normals_list[k]
            z_axis /= np.linalg.norm(z_axis)

            # Project global X to tangent plane
            x_proj = global_x - np.dot(global_x, z_axis)*z_axis
            if np.linalg.norm(x_proj) < 1e-6:
                alt_ref = np.array([0.0, 1.0, 0.0])
                x_proj = alt_ref - np.dot(alt_ref, z_axis)*z_axis

            x_axis = x_proj / np.linalg.norm(x_proj)
            y_axis = np.cross(z_axis, x_axis)
            y_axis /= np.linalg.norm(y_axis)
            x_axis = np.cross(y_axis, z_axis)
            x_axis /= np.linalg.norm(x_axis)

            frames.append((p, x_axis, y_axis, z_axis))

        # 6D pose: xyz + ABC + quaternion
        poses_6d = []
        for origin, x_a, y_a, z_a in frames:
            # Rotation matrix
            R_mat = np.column_stack([x_a, y_a, z_a])
            # 4x4 homojen matris
            hom_mat = np.eye(4)
            hom_mat[0:3,0:3] = R_mat
            # Quaternion hesapla
            q = quaternion_from_matrix(hom_mat)
            # Euler açı debug için
            rpy = self.rotation_matrix_to_rpy(R_mat)
            poses_6d.append([*origin, *rpy, *q])  # xyz + ABC + quat
        return poses_6d

    # -----------------------------
    # Euler açı hesap (A≈pi, B≈0, C≈0 olacak şekilde)
    # -----------------------------
    def rotation_matrix_to_rpy(self, R_mat):
        sy = np.sqrt(R_mat[0,0]**2 + R_mat[1,0]**2)
        singular = sy < 1e-6
        if not singular:
            x = np.arctan2(R_mat[2,1], R_mat[2,2])
            y = np.arctan2(-R_mat[2,0], sy)
            z = np.arctan2(R_mat[1,0], R_mat[0,0])
        else:
            x = np.arctan2(-R_mat[1,2], R_mat[1,1])
            y = np.arctan2(-R_mat[2,0], sy)
            z = 0
        # A≈pi normalize
        if y < 0:
            x += np.pi
        return x, y, z

    # -----------------------------
    # Debug: xyz + ABC + quat
    # -----------------------------
    def print_waypoints(self, poses_6d):
        print("Debug waypoints (x[m], y[m], z[m], A[rad], B[rad], C[rad], quat[x,y,z,w]):")
        for p in poses_6d:
            xyz = p[0:3]
            abc = p[3:6]
            quat = p[6:10]
            print(f"{xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}, "
                  f"{abc[0]:.4f}, {abc[1]:.4f}, {abc[2]:.4f}, "
                  f"{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}")

def main():
    rclpy.init()
    node = CartesianZigZagDebug()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()