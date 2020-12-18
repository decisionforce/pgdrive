import queue
from direct.controls.InputState import InputState
from panda3d.core import Vec3, Camera

from pgdrive.scene_creator.ego_vehicle.base_vehicle import BaseVehicle
from pgdrive.world.pg_world import PgWorld


class ChaseCamera:
    """
    Only chase vehicle now
    """
    queue_length = 3
    TASK_NAME = "update main camera"

    def __init__(
        self, camera: Camera, vehicle: BaseVehicle, camera_height: float, camera_dist: float, pg_world: PgWorld
    ):
        self.camera = camera
        self.camera_queue = None
        self.camera_height = camera_height
        self.camera_dist = camera_dist
        self.light = pg_world.light  # light position is updated with the chase camera when control vehicle
        self.inputs = InputState()
        self.inputs.watchWithModifiers('up', 'k')
        self.inputs.watchWithModifiers('down', 'j')
        self.reset(vehicle, pg_world)

    def renew_camera_place(self, vehicle, task):
        if self.inputs.isSet("up"):
            self.camera_height += 0.5
        if self.inputs.isSet("down"):
            self.camera_height -= 0.5
        self.camera_queue.put(vehicle.chassis_np.get_pos())
        camera_pos = list(self.camera_queue.get())
        camera_pos[2] += self.camera_height
        forward_dir = vehicle.system.get_forward_vector()
        camera_pos[0] += -forward_dir[0] * self.camera_dist
        camera_pos[1] += -forward_dir[1] * self.camera_dist
        self.camera.setPos(*camera_pos)
        current_pos = vehicle.chassis_np.getPos()
        current_pos[2] += 2
        self.camera.lookAt(current_pos)
        if self.light is not None:
            self.light.step(current_pos)
        return task.cont

    def reset(self, vehicle, pg_world):
        """
        Use this function to chase a new vehicle !
        :param vehicle: Vehicle to chase
        :param pg_world: pg_world class
        :return: None
        """
        pos = vehicle.position
        if pg_world.taskMgr.hasTaskNamed(self.TASK_NAME):
            pg_world.taskMgr.remove(self.TASK_NAME)
        pg_world.taskMgr.add(self.renew_camera_place, self.TASK_NAME, extraArgs=[vehicle], appendTask=True)
        self.camera_queue = queue.Queue(self.queue_length)
        for i in range(self.queue_length - 1):
            self.camera_queue.put(Vec3(pos[0], -pos[1], 0))

    def destroy(self, pg_world):
        if pg_world.taskMgr.hasTaskNamed(self.TASK_NAME):
            pg_world.taskMgr.remove(self.TASK_NAME)
