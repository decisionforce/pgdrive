from pgdrive.component.vehicle_module.base_camera import BaseCamera
from pgdrive.constants import CamMask
from pgdrive.engine.engine_utils import get_global_config, engine_initialized


class RGBCamera(BaseCamera):
    # shape(dim_1, dim_2)
    BUFFER_W = 84  # dim 1
    BUFFER_H = 84  # dim 2
    CAM_MASK = CamMask.RgbCam

    def __init__(self):
        assert engine_initialized(), "You should initialize engine before adding camera to vehicle"
        config = get_global_config()["vehicle_config"]["rgb_camera"]
        self.BUFFER_W, self.BUFFER_H = config[0], config[1]
        super(RGBCamera, self).__init__()
        cam = self.get_cam()
        lens = self.get_lens()
        cam.lookAt(0, 2.4, 1.3)
        lens.setFov(60)
        lens.setAspectRatio(2.0)
