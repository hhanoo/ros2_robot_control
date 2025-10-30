import numpy as np
import pyrealsense2 as rs

from .logger import Logger


class RSSensor:

    def __init__(self, sensor_index=0):
        """
        Initialize RealSense sensor with a specific serial number.

        Parameters:
        - sensor_index: Index of the sensor to use
        """
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.intr_params = None
        self.isRunning = False
        self.device_list = self._get_connected_devices()
        self.device_index = sensor_index

        if not self.device_list:
            raise RuntimeError(Logger.error(f"No RealSense devices found!"))

        self.serial_number = self.device_list[self.device_index]
        print(Logger.info(f"Attempting to connect to device: {self.serial_number}"))
        self.config.enable_device(self.serial_number)

    @staticmethod
    def _get_connected_devices():
        """
        Retrieve the list of connected RealSense devices.

        Returns:
        - List of connected RealSense devices (list)
        """
        context = rs.context()
        devices = context.query_devices()
        return [dev.get_info(rs.camera_info.serial_number) for dev in devices]

    def get_device_sn(self):
        """
        Retrieve and print the device serial number.

        Returns:
        - Device serial number (str)
        """
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        serial_number = device.get_info(rs.camera_info.serial_number)
        print(Logger.info(f"Device Serial Number: {serial_number}"))
        return serial_number

    def init(self):
        """
        Initialize the sensor and retrieve intrinsic parameters.

        If the connection fails, try the next available device.

        Returns:
        - True if the sensor is running, False otherwise
        """
        while self.device_index < len(self.device_list):
            try:
                # re-create config each attempt (재시도마다 config 새로 생성)
                self.config = rs.config()
                self.config.enable_device(self.serial_number)

                # enable streams (스트림 설정)
                self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
                self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

                # start pipeline (파이프라인 시작)
                profile = self.pipeline.start(self.config)

                # fetch intrinsics (내부 파라미터)
                color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
                self.intr_params = color_stream.get_intrinsics()
                self.isRunning = True
                print(Logger.info(f"Camera {self.serial_number} started."))
                return True

            except Exception as e:
                print(Logger.error(f"Failed to connect to device {self.serial_number}: {e}"))

                # move to next device (다음 장치로 전환)
                self.device_index += 1
                if self.device_index < len(self.device_list):
                    self.serial_number = self.device_list[self.device_index]
                    print(Logger.info(f"Retrying with device: {self.serial_number}"))
                else:
                    print(Logger.error(f"No available RealSense devices. Exiting."))
                    self.isRunning = False
                    return False  # 모든 장치 연결 실패

        return False  # 모든 장치 연결 실패

    def get_data(self, return_rgb=True, return_depth=True):
        """
        Retrieve RGB and depth data from the sensor.

        Parameters:
        - return_rgb(bool): Whether to return RGB data
        - return_depth(bool): Whether to return depth data

        Returns:
        - RGB data (numpy.ndarray)
        - Depth data (numpy.ndarray)
        """
        if not self.isRunning:
            print(Logger.error(f"Camera pipeline is not running. Cannot get data."))
            return None, None

        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not return_rgb:
                rgb_data = None
            else:
                rgb_data = np.asanyarray(color_frame.get_data())

            if not depth_frame or not return_depth:
                depth_data = None
            else:
                depth_data = np.asanyarray(depth_frame.get_data())

            return rgb_data, depth_data

        except Exception as e:
            print(Logger.error(f"Pipeline error: {e}"))
            self.isRunning = False  # Mark as not running if error occurs
            return None, None

    def stop(self):
        """
        Stop the sensor.

        Returns:
        - None
        """
        if self.isRunning:
            try:
                self.pipeline.stop()
                print(Logger.info(f"Camera {self.serial_number} stopped."))
            except Exception as e:
                print(Logger.error(f"Error stopping pipeline: {e}"))
        else:
            print(Logger.warning(f"Camera {self.serial_number} was not running."))

        self.isRunning = False

    def get_camera_parameters(self):
        """
        Get camera parameters.

        Returns:
        - Camera matrix (numpy.ndarray)
        - Camera parameters (tuple) (fx, fy, ppx, ppy)
        """
        if self.intr_params is None:
            raise RuntimeError(Logger.error(f"Intrinsic parameters are not available. Start the camera first."))

        ppx, ppy, fx, fy = self.intr_params.ppx, self.intr_params.ppy, self.intr_params.fx, self.intr_params.fy
        camera_matrix = np.array([[fx, 0, ppx], [0, fy, ppy], [0, 0, 1]])
        camera_params = (camera_matrix[0, 0], camera_matrix[1, 1], camera_matrix[0, 2], camera_matrix[1, 2])
        return camera_matrix, camera_params


if __name__ == "__main__":
    """
    Standalone test for RSSensor (ROS2-independent)
    ------------------------------------------------
    How to run (for standalone testing in ROS2 environment):
        cd /ros2_ws
        PYTHONPATH=/ros2_ws/src python3 -m vision_bridge.vision_bridge.rs_sensor
    """

    import cv2
    import matplotlib.pyplot as plt

    def get_color_depth_data_fast(depth_data: np.ndarray, max_range: float = 3000.0) -> np.ndarray:
        """
        Convert depth data to a color-mapped RGB image (fast vectorized version).

        Parameters:
            depth_data (np.ndarray): 2D depth array (float or uint16)
            max_range (float): maximum depth range for color normalization

        Returns:
            np.ndarray: colorized depth image (H, W, 3) in uint8
        """
        # 1. Normalize depth values -> 0~1
        depth_norm = np.clip(depth_data / max_range, 0, 1)

        # 2. 0~255 mapping -> uint8 index
        depth_idx = (depth_norm * 255).astype(np.uint8)

        # 3. matplotlib jet colormap
        cmap = (plt.cm.jet(np.arange(256))[:, :3] * 255).astype(np.uint8)  # shape (256, 3)

        # 4. color map indexing (vectorized)
        color_depth = cmap[depth_idx]

        # 5. depth=0(no data) pixel is black
        color_depth[depth_data <= 0] = (0, 0, 0)

        return color_depth

    # 0. connect to sensor
    sensor = RSSensor(sensor_index=0)

    # 1. initialize sensor
    if sensor.init():
        sensor_matrix, _ = sensor.get_camera_parameters()
        print(Logger.info(f"Camera matrix:\n{sensor_matrix}"))

    else:
        print(Logger.error(f"Failed to initialize sensor."))
        exit()

    # 2. get data
    while True:
        rgb_data, depth_data = sensor.get_data(return_rgb=True, return_depth=True)

        if rgb_data is not None:
            cv2.imshow("RGB", rgb_data)
        else:
            print(Logger.error(f"Failed to get RGB data."))

        if depth_data is not None:
            color_depth_data = get_color_depth_data_fast(depth_data)
            cv2.imshow("Depth", color_depth_data)
        else:
            print(Logger.error(f"Failed to get depth data."))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 3. stop sensor
    sensor.stop()
    cv2.destroyAllWindows()
