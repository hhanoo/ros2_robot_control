import socket
import time
import threading
import struct
import time
import threading
import numpy as np

from ur_bridge_py.logger import Logger


class URClass():

    def __init__(self, ip="127.0.0.1"):
        """
        Initialize UR robot connection and internal state variables.

        Parameters:
            ip (str): UR IP address
            use_thread (bool): If True, use internal thread for auto-update.
                               If False, robot_info() must be called manually.
        """

        # Connection setup
        self.UR_IP = ip
        self.UR_PORT = 30003
        self.connected = False

        # Socket setup
        self.socket = None
        self.buffer_size = 1220

        # Robot state variables
        self.act_q = np.zeros(6, dtype=np.float32)  # Actual joint position [rad]
        self.act_X = np.zeros(6, dtype=np.float32)  # Actual TCP pose [m, rad]
        self.act_T = np.zeros((4, 4), dtype=np.float32)  # Actual transformation matrix
        self.des_q = np.zeros(6, dtype=np.float32)  # Desired joint position
        self.des_X = np.zeros(6, dtype=np.float32)  # Desired TCP pose [m, rad]
        self.des_T = np.zeros((4, 4), dtype=np.float32)  # Desired transformation matrix

        # Orientation (rotation) parameters
        self.R = np.zeros((3, 3), dtype=np.float32)  # Rotation matrix

        # Force/Torque sensing
        self.act_F = np.zeros(6, dtype=np.float32)  # Actual EE force/moment [Fx, Fy, Fz, Mx, My, Mz]

        # Digital I/O
        self.n_digital_io = 8
        self.digital_input = [False] * self.n_digital_io  # Digital input state array (8 bits)

        # System state flags
        self.mode = 0  # Robot mode
        self.safety_mode = 0  # Safety mode
        self.prog_state = 0  # Program state
        self.robot_state = 0  # [-1: error(include failed to execute motion), 0: idle, 1: moving]

        # Motion parameters
        self.velocity_percentage = 100.

        # Start background status thread
        self.update_thread = None
        self.send_move_flag = False

    def connect(self):
        """
        Connect to UR robot controller via TCP/IP socket.
        """

        if self.is_connected():
            print(Logger.info("Already connected."))
            return True

        try:
            # Create socket and connect to robot
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.UR_IP, self.UR_PORT))
            self.connected = True
            print(Logger.info(f"Connection successful: {self.UR_IP}:{self.UR_PORT}"))

            # Check if update thread is already running
            if self.update_thread is not None and self.update_thread.is_alive():
                print(Logger.info("Status thread already running."))
            # Start background thread for status updates
            else:
                self.update_thread = threading.Thread(target=self._update_info, daemon=True)
                self.update_thread.start()
                return True

            return True

        except Exception as e:
            print(Logger.error(f"Connection failed: {e}"))
            self.connected = False
            if self.socket is not None:
                self.socket.close()
                self.socket = None
            return False

    def disconnect(self):
        """
        Disconnect from UR robot controller.
        """
        if not self.is_connected():
            print(Logger.info("Already disconnected."))
            return False

        try:
            # Update connection status
            self.connected = False
            time.sleep(0.05)

            # Close socket and stop update thread
            if self.socket is not None:
                self.socket.close()
                self.socket = None

            print(Logger.info(f"Disconnection successful: {self.UR_IP}:{self.UR_PORT}"))
            return True
        except Exception as e:
            print(Logger.error(f"Disconnection failed: {e}"))
            return False

    def is_connected(self):
        """
        Check if connection to robot is alive.

        Returns:
            bool: True if connected, False otherwise.
        """
        if self.socket is None:
            return False
        return self.update_thread is not None and self.update_thread.is_alive() and self.connected

    def movej(self, des_q, acc=None, vel=None, duration=None, mode='abs'):
        """
        Move the robot in joint space using the UR 'moveJ' command.
        
        Parameters:
            des_q (array-like): Target joint position [rad], relative or absolute.
            acc (float): Acceleration
            vel (float): Velocity
            duration (float): Motion duration [s].
            mode (str): 'abs' for absolute move, 'rel' for relative move.
        """

        # Initialize command joint vector and velocity
        q_cmd = np.zeros(6, dtype=np.float32)

        # Set default values if not provided
        if acc is None:
            acc = self.velocity_percentage * 4 / 100
        if vel is None:
            vel = self.velocity_percentage * 1 / 100
        if duration is None:
            duration = 0.0

        # Compute target position
        if mode == 'rel':
            q_cmd = self.des_q + np.array(des_q, dtype=np.float32)
        elif mode == 'abs':
            q_cmd = np.array(des_q, dtype=np.float32)
        else:
            print(Logger.error(f"moveJ: Invalid mode ({mode}). Must be 'abs' or 'rel'."))
            return

        # Build command string
        cmd = (f"movej([{q_cmd[0]},{q_cmd[1]},{q_cmd[2]},{q_cmd[3]},{q_cmd[4]},{q_cmd[5]}],"
               f"a={acc},v={vel},t={duration},r=0)\n")

        # Send command via TCP socket
        cmd_byte = str.encode(cmd)
        self.socket.send(cmd_byte)
        self.send_move_flag = True

    def movel_pose(self, des_X, acc=None, vel=None, duration=None, mode='abs'):
        """
        Move the robot linearly in Cartesian (TCP) space using the UR 'moveL' command.

        Parameters:
            des_X (array-like): Target TCP pose [x, y, z, rx, ry, rz], relative or absolute.
            acc (float): Acceleration
            vel (float): Velocity
            duration (float): Motion duration [s].
            mode (str): 'abs' for absolute move, 'rel' for relative move.
        """

        # Initialize TCP command vector
        X_cmd = np.zeros(6, dtype=np.float32)

        # Set default values if not provided
        if acc is None:
            acc = self.velocity_percentage * 3.14 / 100
        if vel is None:
            vel = self.velocity_percentage * 3.14 / 100
        if duration is None:
            duration = 0.0

        # Compute target TCP pose
        if mode == 'rel':
            X_cmd = self.des_X + np.array(des_X, dtype=np.float32)
        elif mode == 'abs':
            X_cmd = np.array(des_X, dtype=np.float32)
        else:
            print(Logger.error(f"moveL: Invalid mode ({mode}). Must be 'abs' or 'rel'."))
            return

        # Build command string
        cmd = (f"movel(p[{X_cmd[0]},{X_cmd[1]},{X_cmd[2]},{X_cmd[3]},{X_cmd[4]},{X_cmd[5]}],"
               f"a={acc},v={vel},t={duration},r=0)\n")

        # Send command via TCP socket
        cmd_byte = str.encode(cmd)
        self.socket.send(cmd_byte)
        self.send_move_flag = True

    def movel_T(self, T, acc=None, vel=None, duration=None, mode='abs'):
        """
        Move the robot linearly using a 16-element homogeneous transformation array.

        Parameters:
            T (list or np.ndarray): 16-element array representing 4x4 homogeneous transform.
                                    [r11, r12, r13, px,
                                    r21, r22, r23, py,
                                    r31, r32, r33, pz,
                                    0,   0,   0,   1]
            acc (float): Acceleration.
            vel (float): Velocity.
            duration (float): Motion duration [s].
            mode (str): 'abs' for absolute move, 'rel' for relative move.
        """

        # Input validation
        T = np.array(T, dtype=np.float32)
        if T.size != 16:
            print(Logger.error("movel_T: Input must be a 16-element array representing 4x4 matrix."))
            return

        # Reshape into 4x4 matrix
        T = T.reshape(4, 4)

        # Extract position and rotation
        pos = T[0:3, 3]  # [x, y, z]
        R = T[0:3, 0:3]  # 3x3 rotation matrix

        # Convert rotation matrix to rotation vector [rx, ry, rz]
        rx, ry, rz = self._rotationMatrix2RotationVector(R)

        # Combine into TCP pose vector
        des_X = np.array([pos[0], pos[1], pos[2], rx, ry, rz], dtype=np.float32)

        # Use existing movel_pose() function
        self.movel_pose(des_X, acc=acc, vel=vel, duration=duration, mode=mode)

    def wait_move(self, timeout=10.0):
        """
        Wait until robot finishes current motion (blocking).
        
        Parameters:
            interval (float): Check interval in seconds
            timeout (float): Maximum wait time in seconds
        """
        start_time = time.time()

        while self.send_move_flag:
            if self.robot_state == -1:
                print(Logger.error("Robot is in error state."))
                break
            elif self.robot_state == 1:
                self.send_move_flag = False
                break
            time.sleep(0.01)

        while True:
            if self.robot_state == 0:  # Idle state
                print(Logger.info("Robot is idle. Motion finished."))
                break
            if time.time() - start_time > timeout:
                print(Logger.warning(f"wait_move timeout after {timeout} seconds"))
                break

    def set_velocity(self, velocity):
        """
        Set the velocity of the robot.

        Parameters:
            velocity (float): Velocity [% of max velocity].
        """

        # Validate velocity
        if velocity < 0.0:
            velocity = 0.0
        elif velocity > 100.0:
            velocity = 100.0

        self.velocity_percentage = velocity

    def stopj(self, acc=None):
        """
        Stop the robot's joint motion using the UR 'stopj' command.

        Parameters:
            acc (float): Deceleration. If None, default scaling based on current velocity.
        """
        if acc is None:
            acc = self.velocity_percentage * 4 / 100

        # Build command string
        cmd = f"stopj({acc})\n"

        # Send command via TCP socket
        cmd_byte = str.encode(cmd)
        self.socket.send(cmd_byte)

    def stopl(self, acc=None):
        """
        Stop the robot's linear (Cartesian) motion using the UR 'stopl' command.

        Parameters:
            acc (float): Deceleration. If None, default scaling based on current velocity.
        """
        if acc is None:
            acc = self.velocity_percentage * 3.14 / 100

        # Build command string
        cmd = f"stopl({acc})\n"

        # Send command via TCP socket
        cmd_byte = str.encode(cmd)
        self.socket.send(cmd_byte)

    def controlbox_digital_out(self, port: int, value: bool):
        """
        Set a standard digital output signal on the UR controller.

        Parameters:
            port (int): Digital output port number (0-7).
            value (bool): Output value (True = ON, False = OFF).
        """

        # Validation
        if port < 0 or port >= self.n_digital_io:
            print(Logger.error(f"controlbox_digital_out: Invalid port number ({port}). Must be 0-{self.n_digital_io-1}."))
            return

        # Build command string
        cmd = f"set_standard_digital_out({port},{value})\n"

        # Send command via TCP socket
        cmd_byte = str.encode(cmd)
        self.socket.send(cmd_byte)

    def controlbox_digital_in(self, port: int):
        """
        Get a standard digital input signal from the UR controller.

        Parameters:
            port (int): Digital input port number (0-7).
        """

        # Validation
        if port < 0 or port >= self.n_digital_io:
            print(Logger.error(f"controlbox_digital_in: Invalid port number ({port}). Must be 0-{self.n_digital_io-1}."))
            return

        return self.digital_input[port]

    def robot_info(self):
        """
            Continuously receive robot status data via TCP and update internal state variables.

            Updates:
                - act_q, des_q
                - act_X, des_X
                - angle_axis_angle, angle_axis_axis
                - R
        """
        try:
            msg = self.socket.recv(self.buffer_size)
            if len(msg) > 1108:
                # Header check (UR RTDE packet signature)
                if msg[0:3] == b'\x00\x00\x04':

                    # Unpack desired joint & TCP positions
                    _des_q = [struct.unpack('>d', msg[12 + 8 * i:20 + 8 * i])[0] for i in range(6)]
                    _des_X = [struct.unpack('>d', msg[588 + 8 * i:596 + 8 * i])[0] for i in range(6)]

                    # Unpack actual joint & TCP positions
                    _act_q = [struct.unpack('>d', msg[252 + 8 * i:260 + 8 * i])[0] for i in range(6)]
                    _act_X = [struct.unpack('>d', msg[444 + 8 * i:452 + 8 * i])[0] for i in range(6)]

                    # Unpack force/torque sensing
                    _act_F = [struct.unpack('>d', msg[540 + 8 * i:548 + 8 * i])[0] for i in range(6)]

                    # Update robot information
                    self.des_q = _des_q
                    self.act_q = _act_q
                    self.des_X = _des_X
                    self.act_X = _act_X
                    self.act_F = _act_F

                    # Unpack digital inputs
                    _digital_input = int(struct.unpack('>d', msg[684:692])[0])
                    _mask_bit = [1 << i for i in range(self.n_digital_io)]
                    self.digital_input = [bool(_digital_input & bit) != 0 for bit in _mask_bit]

                    # Unpack system modes
                    self.mode = struct.unpack('>d', msg[756:764])[0]
                    self.safety_mode = struct.unpack('>d', msg[812:820])[0]
                    self.prog_state = struct.unpack('>d', msg[1052:1060])[0]

                    # Update robot state
                    if self.prog_state == 2.0:  # Moving state
                        self.robot_state = 1
                    elif self.prog_state == 1.0:  # Idle state
                        self.robot_state = 0
                    else:
                        self.robot_state = -1  # Error or Unsafe state

                    # Rotation vector [rx, ry, rz]
                    _rx, _ry, _rz = _act_X[3:6]

                    # Update rotation matrix using helper function
                    self.R = self._rotationVector2RotationMatrix(_rx, _ry, _rz)

                    # Update transformation matrix (4x4)
                    _act_T = np.eye(4, dtype=np.float32)
                    _act_T[0:3, 0:3] = self.R
                    _act_T[0:3, 3] = np.array(_act_X[0:3], dtype=np.float32)  # translation vector
                    self.act_T = _act_T

                    # Update desired transformation matrix (4x4)
                    _des_T = np.eye(4, dtype=np.float32)
                    _des_T[0:3, 0:3] = self._rotationVector2RotationMatrix(_des_X[3], _des_X[4], _des_X[5])
                    _des_T[0:3, 3] = np.array(_des_X[0:3], dtype=np.float32)  # translation vector
                    self.des_T = _des_T
            else:
                print(Logger.warning("Invalid message length or header."))

        except Exception as e:
            if not self.connected:
                print(Logger.info("Robot monitoring stopped (manual disconnect)."))

            print(Logger.error(f"robot_info: {e}. Reconnecting..."))
            self.connected = False

            try:
                self.socket.close()
                time.sleep(1)
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.UR_IP, self.UR_PORT))
                self.connected = True
            except:
                time.sleep(5)

    def print_robot_mode(self):
        """
        Print the current robot mode in human-readable text.

        Mode codes:
            -1 : No controller
             0 : Disconnected
             1 : Confirm safety
             2 : Booting
             3 : Power off
             4 : Power on
             5 : Idle
             6 : Backdrive
             7 : Running (ready to move or moving)
             8 : Updating firmware
        """

        mode_dict = {
            -1: "ROBOT_MODE_NO_CONTROLLER (-1)",
            0: "ROBOT_MODE_DISCONNECTED (0)",
            1: "ROBOT_MODE_CONFIRM_SAFETY (1)",
            2: "ROBOT_MODE_BOOTING (2)",
            3: "ROBOT_MODE_POWER_OFF (3)",
            4: "ROBOT_MODE_POWER_ON",
            5: "ROBOT_MODE_IDLE (5)",
            6: "ROBOT_MODE_BACKDRIVE",
            7: "ROBOT_MODE_RUNNING (7)",
            8: "ROBOT_MODE_UPDATE_FIRMWARE (8)"
        }

        # Get the mode name from dictionary
        mode_name = mode_dict.get(int(self.mode), f"UNKNOWN_MODE ({self.mode})")

        return mode_name

    def _update_info(self):
        while self.connected:
            self.robot_info()

    def _rad2deg(self, rad):
        """
        Convert radians to degrees.

        Parameters:
            rad (float): Angle in radians.

        Returns:
            float: Angle in degrees.
        """
        return np.degrees(rad)

    def _deg2rad(self, deg):
        """
        Convert degrees to radians.

        Parameters:
            deg (float): Angle in degrees.

        Returns:
            float: Angle in radians.
        """
        return np.radians(deg)

    def _Rx(self, q):
        """
        Generate a 3x3 rotation matrix for rotation about the X-axis.

        Parameters:
            q (float): Rotation angle in degrees.

        Returns:
            np.ndarray: 3x3 rotation matrix (float32).
        """

        rad = np.radians(q)  # convert degree → radian
        R = np.array([[1., 0., 0.], [0., np.cos(rad), -np.sin(rad)], [0., np.sin(rad), np.cos(rad)]], dtype=np.float32)

        return R

    def _Ry(self, q):
        """
        Generate a 3x3 rotation matrix for rotation about the Y-axis.

        Parameters:
            q (float): Rotation angle in degrees.

        Returns:
            np.ndarray: 3x3 rotation matrix (float32).
        """

        rad = np.radians(q)  # convert degree → radian
        R = np.array([[np.cos(rad), 0., np.sin(rad)], [0., 1., 0.], [-np.sin(rad), 0., np.cos(rad)]], dtype=np.float32)

        return R

    def _Rz(self, q):
        """
        Generate a 3x3 rotation matrix for rotation about the Z-axis.

        Parameters:
            q (float): Rotation angle in degrees.

        Returns:
            np.ndarray: 3x3 rotation matrix (float32).
        """

        rad = np.radians(q)  # convert degree → radian
        R = np.array([[np.cos(rad), -np.sin(rad), 0.], [np.sin(rad), np.cos(rad), 0.], [0., 0., 1.]], dtype=np.float32)

        return R

    def _rotationVector2RotationMatrix(self, rx, ry, rz):
        """
        Convert a rotation vector [rx, ry, rz] to a 3x3 rotation matrix.

        Parameters:
            rx, ry, rz (float): Rotation vector components [rad].

        Returns:
            np.ndarray: 3x3 rotation matrix (float32).
        """

        # Rotation vector → rotation angle (θ)
        theta = np.sqrt(rx**2 + ry**2 + rz**2)

        # Compute the rotation axis (unit vector) if theta > 0.01
        if np.abs(theta) > 0.01:
            axis = np.array([rx / theta, ry / theta, rz / theta], dtype=np.float32)
        else:
            theta = 0.
            axis = np.zeros(3, dtype=np.float32)

        # Compute trigonometric terms used in Rodrigues’ formula
        _c = np.cos(theta)
        _s = np.sin(theta)
        _C = 1.0 - np.cos(theta)

        # Extract axis components for clarity (ux, uy, uz)
        _ux = axis[0]
        _uy = axis[1]
        _uz = axis[2]

        # Apply Rodrigues’ formula to compute the rotation matrix
        R = np.array([[_ux * _ux * _C + _c, _ux * _uy * _C - _uz * _s, _ux * _uz * _C + _uy * _s],
                      [_uy * _ux * _C + _uz * _s, _uy * _uy * _C + _c, _uy * _uz * _C - _ux * _s],
                      [_uz * _ux * _C - _uy * _s, _uz * _uy * _C + _ux * _s, _uz * _uz * _C + _c]],
                     dtype=np.float32)

        return R

    def _rotationMatrix2RotationVector(self, R):
        """
        Convert a 3x3 rotation matrix to a rotation vector [rx, ry, rz].

        Parameters:
            R (np.ndarray): 3x3 rotation matrix.

        Returns:
            tuple: (rx, ry, rz) rotation vector [rad].
        """

        # Handle near-180° rotation case
        if ((R[0][0] == -1 and R[1][1] == -1) or \
            (R[0][0] == -1 and R[2][2] == -1) or \
            (R[1][1] == -1 and R[2][2] == -1)):
            rx = np.pi / 2. * (R[0][0] + 1.)
            ry = np.pi / 2. * (R[1][1] + 1.)
            rz = np.pi / 2. * (R[2][2] + 1.)
        # Handle zero rotation (no rotation, identity matrix)
        elif (R[0][0] == 1 and R[1][1] == 1 and R[2][2] == 1):
            rx = 0.
            ry = 0.
            rz = 0.
        # General case
        else:
            lx = R[2][1] - R[1][2]
            ly = R[0][2] - R[2][0]
            lz = R[1][0] - R[0][1]

            # Axis vector norm
            axis_norm = np.sqrt(lx**2 + ly**2 + lz**2)

            # Rotation angle
            theta = np.arctan2(axis_norm, R[0][0] + R[1][1] + R[2][2] - 1.)

            # Normalize and scale
            if axis_norm > 0.:
                rx = theta / axis_norm * lx
                ry = theta / axis_norm * ly
                rz = theta / axis_norm * lz
            else:
                rx, ry, rz = 0.0, 0.0, 0.0

        return rx, ry, rz


if __name__ == '__main__':
    """
    Standalone test for URClass (ROS2-independent)
    ------------------------------------------------
    How to run (for standalone testing in ROS2 environment):
        cd /ros2_ws
        PYTHONPATH=/ros2_ws/src/robots/ur_bridge_py python3 -m ur_bridge_py.ur_class
    """

    # 0. connect to robot
    robot = URClass(ip="127.0.0.1")
    robot.connect()

    # initial delay
    time.sleep(3)
    print(Logger.info('[Robot] Robot ready'))

    # 1. show desired/current joint position [rad]
    print(Logger.info(f"Desired joint position [rad]: {robot.des_q}"))
    print(Logger.info(f"Actual joint position [rad]: {robot.act_q}"))

    # 2. show desired/current workspace position [m, rotation vector]
    print(Logger.info(f"Desired workspace position [x, y, z, rx, ry, rz]: {robot.des_X}"))
    print(Logger.info(f"Actual workspace position [x, y, z, rx, ry, rz]: {robot.act_X}"))

    # 3. show desired/current transformation matrix
    print(Logger.info(f"Desired transformation matrix:\n {robot.des_T}"))
    print(Logger.info(f"Actual transformation matrix:\n {robot.act_T}"))

    # 4. joint motion, absolute [deg]
    robot.movej([-70. * np.pi / 180., -60. * np.pi / 180., -130. * np.pi / 180., \
                 -80. * np.pi / 180., 90. * np.pi / 180., 20. * np.pi / 180.])
    robot.wait_move()

    # 5. joint motion, relative [rad] -> relative motion of the joint angles [rad]
    robot.movej([0., 0., 0., 0., 1., 0.], mode='rel')
    robot.wait_move()

    # 6. workspace motion, relative [m, rotation vector]
    robot.movel_pose([0, 0, -0.15, 0, 0, 0], mode='rel')
    robot.wait_move()

    # 7. control digital output: set DIO to False
    robot.controlbox_digital_out(0, False)
    time.sleep(1)
    robot.controlbox_digital_out(0, True)
    time.sleep(1)
    robot.controlbox_digital_out(0, False)
    time.sleep(1)

    # 8. disconnect from robot
    robot.disconnect()
