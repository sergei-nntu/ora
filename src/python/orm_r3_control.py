from pathlib import Path

import numpy as np
from ikpy.chain import Chain
from osp import OSP
from scipy.spatial.transform import Rotation


class ORMR3Control:
    OSP_ANGLE_UNITS_PER_REVOLUTION = 32768

    def __init__(self, serial_port_path=None):
        self.osp = OSP(serial_port_path) if serial_port_path is not None else None

        urdf_path = Path(__file__).with_name("orm_r3.urdf")
        self.chain = Chain.from_urdf_file(str(urdf_path),base_elements=["base_link"])
        self.joint_angles = np.zeros(len(self.chain.links))
        print("Point Angles: "+str(self.joint_angles))
        self.calculate_end_effector_pose()

    def calculate_end_effector_pose(self):
        """Calculate the tool pose for the current joint angles.

        Position is returned as XYZ in metres and orientation as extrinsic XYZ
        Euler angles (roll, pitch, yaw) in radians.
        """
        self.end_effector_transform = self.chain.forward_kinematics(
            self.joint_angles
        )
        self.end_effector_position = self.end_effector_transform[:3, 3].copy()
        self.end_effector_rotation_matrix = self.end_effector_transform[
            :3, :3
        ].copy()
        self.end_effector_euler_angles = Rotation.from_matrix(
            self.end_effector_rotation_matrix
        ).as_euler("xyz")
        return self.end_effector_position, self.end_effector_euler_angles

    def set_joints_angles(self, angles):
        """Set joint angles in radians and print the anticipated tool pose.

        ``angles`` may contain the six revolute-joint angles or IKPy's complete
        chain vector, which also includes the origin and fixed tool link.
        """
        angles = np.asarray(angles, dtype=float).reshape(-1)
        revolute_indices = [
            index
            for index, link in enumerate(self.chain.links)
            if link.joint_type == "revolute"
        ]

        if len(angles) == len(revolute_indices):
            joint_angles = np.zeros(len(self.chain.links), dtype=float)
            joint_angles[revolute_indices] = angles
        elif len(angles) == len(self.chain.links):
            joint_angles = angles.copy()
        else:
            raise ValueError(
                f"Expected {len(revolute_indices)} revolute-joint angles or "
                f"{len(self.chain.links)} complete-chain angles, got "
                f"{len(angles)}"
            )

        self.joint_angles = joint_angles
        anticipated_position, anticipated_euler_angles = (
            self.calculate_end_effector_pose()
        )
        anticipated_x, anticipated_y, anticipated_z = anticipated_position
        anticipated_roll, anticipated_pitch, anticipated_yaw = (
            anticipated_euler_angles
        )

        print(
            "Anticipated pose: "
            f"x={anticipated_x:.6f}, y={anticipated_y:.6f}, "
            f"z={anticipated_z:.6f}, pitch={anticipated_pitch:.6f}, "
            f"roll={anticipated_roll:.6f}, yaw={anticipated_yaw:.6f}"
        )
        return anticipated_position, anticipated_euler_angles

    def set_pose(self, x, y, z, pitch=None, roll=None, yaw=None,
                 command="set_angle"):
        """Move the end effector to an XYZ pose using inverse kinematics.

        Position is expressed in metres and orientation in radians. If no
        orientation is supplied, IK is solved for position only. If only some
        orientation components are supplied, the others retain their current
        values. Returns the target chain angles, achieved XYZ position, and
        achieved XYZ Euler angles (roll, pitch, yaw).
        """
        if command not in ("set_angle", "set_coarse_angle"):
            raise ValueError("command must be set_angle or set_coarse_angle")
        target_position = np.array([x, y, z], dtype=float)
        orientation = (roll, pitch, yaw)

        if all(angle is None for angle in orientation):
            target_angles = self.chain.inverse_kinematics(
                target_position=target_position,
                initial_position=self.joint_angles,
            )
        else:
            current_orientation = self.end_effector_euler_angles
            target_euler_angles = np.array(
                [
                    current if requested is None else requested
                    for requested, current in zip(
                        orientation, current_orientation
                    )
                ],
                dtype=float,
            )
            target_orientation = Rotation.from_euler(
                "xyz", target_euler_angles
            ).as_matrix()
            target_angles = self.chain.inverse_kinematics(
                target_position=target_position,
                target_orientation=target_orientation,
                orientation_mode="all",
                initial_position=self.joint_angles,
            )

        target_angles = np.asarray(target_angles, dtype=float)
        osp_scale = self.OSP_ANGLE_UNITS_PER_REVOLUTION / (2 * np.pi)
        joint_address = 0

        for link, angle in zip(self.chain.links, target_angles):
            print("Link: "+str(link)+"; Angle: "+str(angle))
            if link.joint_type != "revolute":
                continue

            if self.osp is not None:
                osp_angle = int(round(float(angle) * osp_scale))
                if command == "set_coarse_angle":
                    self.osp.ora_set_coarse_angle(joint_address, osp_angle)
                else:
                    self.osp.ora_set_angle(joint_address, osp_angle)
            joint_address += 1

        self.joint_angles = target_angles
        achieved_position, achieved_euler_angles = (
            self.calculate_end_effector_pose()
        )
        achieved_x, achieved_y, achieved_z = achieved_position
        achieved_roll, achieved_pitch, achieved_yaw = achieved_euler_angles

        print(
            "Achieved pose: "
            f"x={achieved_x:.6f}, y={achieved_y:.6f}, "
            f"z={achieved_z:.6f}, pitch={achieved_pitch:.6f}, "
            f"roll={achieved_roll:.6f}, yaw={achieved_yaw:.6f}"
        )
        return target_angles, achieved_position, achieved_euler_angles
