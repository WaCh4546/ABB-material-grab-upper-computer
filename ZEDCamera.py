from DepthCamera import DepthCamera
import pyzed.sl as sl
import time
import cv2

class ZEDCamera(DepthCamera):
    def __init__(self, cam_number=None, resolution=2200, camera_fps=15, depth_min=400, depth_max=5000):
        """
        There are maybe multi cameras in one computer, in this case it is necessary to specify
        which camera is initialized. Each camera is recognized by its serial number.
        2.2K	4416x1242	15	                Wide
        1080p	3840x1080	30, 15	            Wide
        720p	2560x720	60, 30, 15	        Extra Wide
        WVGA	1344x376	100, 60, 30, 15	    Extra Wide
        :param cam_number: The serial number of the camera.
        :param resolution: HD2K, HD1080, HD720, VGA
        :param camera_fps: 15, 30, 60, 100.
        """
        if resolution == 2200:
            DepthCamera.__init__(self, (2208, 1242))
            resolution = sl.RESOLUTION.HD2K
        elif resolution == 1080:
            DepthCamera.__init__(self, (1920, 1080))
            resolution = sl.RESOLUTION.HD1080
        elif resolution == 720:
            DepthCamera.__init__(self, (1280, 720))
            resolution = sl.RESOLUTION.HD720
        else:
            DepthCamera.__init__(self, (672, 376))
            resolution = sl.RESOLUTION.VGA

        # init camera
        self.camera = sl.Camera()
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use PERFORMANCE depth mode
        init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meter units (for depth measurements)
        init_params.camera_resolution = resolution
        init_params.camera_fps = camera_fps
        init_params.depth_minimum_distance = depth_min
        init_params.depth_maximum_distance = depth_max

        if cam_number is None:
            cam_number = self.get_serial_number(0)
            if cam_number is None:
                raise Exception("Camera does not exist!")

        init_params.set_from_serial_number(cam_number)
        self.camera_number = cam_number

        # 打开摄相机
        err = self.camera.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise Exception("Failed to open ZED camera!")

        # 打开相机后创建并设置运行时参数
        self.runtime_parameters = sl.RuntimeParameters()
        self.runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD
        self.runtime_parameters.confidence_threshold = 100
        self.runtime_parameters.textureness_confidence_threshold = 100

        # 获取相机内参fx,fy,Cx,Cy
        # 通过内参和深度数据,可以计算每个像素点的(x, y)坐标
        calibration_params = self.camera.get_camera_information().calibration_parameters
        self.cam_params = (calibration_params.left_cam.fx,
                           calibration_params.left_cam.fy,
                           calibration_params.left_cam.cx,
                           calibration_params.left_cam.cy)
        # 获取相机畸变系数k1,k2,k3,p1,p2
        self.int_disto=(   calibration_params.left_cam.disto[0],
                           calibration_params.left_cam.disto[1],
                           calibration_params.left_cam.disto[2],
                           calibration_params.left_cam.disto[3],
                           calibration_params.left_cam.disto[4])

        # 创建Mat对象,用于获取图像
        self.left_image = sl.Mat()
        self.right_image = sl.Mat()
        self.point_cloud = sl.Mat()
    def __del__(self):
        self.camera.close()
    def camera_identity(self):
        return str(self.camera_number)

    @staticmethod
    def enum_cameras():
        cameras = sl.Camera.get_device_list()
        cam_numbers = list()
        for cam in cameras:
            cam_numbers.append(cam.serial_number)
        return cam_numbers

    @staticmethod
    def get_serial_number(camera_id):
        cameras = sl.Camera.get_device_list()
        if len(cameras) >= camera_id:
            return cameras[camera_id].serial_number
        else:
            return None

    def refresh(self):
        if self.camera.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.timestamp = time.time()
            self.camera.retrieve_image(self.left_image, sl.VIEW.LEFT)
            self.camera.retrieve_image(self.right_image, sl.VIEW.RIGHT)
            self.camera.retrieve_measure(self.point_cloud, sl.MEASURE.XYZ)

            self.RGBimage = self.left_image.get_data()[:, :, 0:3]   # the last channel, alpha, is discarded
            self.RGBimage_right = self.right_image.get_data()[:, :, 0:3]
            self.Xmap = self.point_cloud.get_data()[:, :, 0]
            self.Ymap = self.point_cloud.get_data()[:, :, 1]
            self.Zmap = self.point_cloud.get_data()[:, :, 2]

            DepthCamera.refresh(self)
            return True
        else:
            return False

if __name__ == '__main__':
    cameras = ZEDCamera.enum_cameras()
    camera1 = ZEDCamera(cameras[0], resolution=720, camera_fps=15)
    #camera2 = ZEDCamera(cameras[1], resolution=sl.RESOLUTION.HD720, camera_fps=15)

    key = 0
    while key != ord('e') and key != ord('E'):
        key = cv2.waitKey(50) & 0xFF
        camera1.test(key)
        #camera2.test(key, width=1400)




