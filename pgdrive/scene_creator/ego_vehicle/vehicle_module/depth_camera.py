from panda3d.core import Vec3, NodePath, Shader, RenderState, ShaderAttrib, BitMask32, PNMImage
from pgdrive.pg_config.cam_mask import CamMask
from panda3d.core import GeoMipTerrain
from pgdrive.utils import is_mac
from pgdrive.utils.asset_loader import AssetLoader
from pgdrive.world.image_buffer import ImageBuffer
from pgdrive.world.pg_world import PgWorld


class DepthCamera(ImageBuffer):
    # shape(dim_1, dim_2)
    BUFFER_X = 84  # dim 1
    BUFFER_Y = 84  # dim 2
    CAM_MASK = CamMask.DepthCam
    display_top = 1.0

    def __init__(self, length: int, width: int, view_ground: bool, chassis_np: NodePath, pg_world: PgWorld):
        """
        :param length: Control resolution of this sensor
        :param width: Control resolution of this sensor
        :param view_ground: Lane line will be invisible when set to True
        :param chassis_np: The vehicle chassis to place this sensor
        :param pg_world: PG-World
        """
        self.view_ground = view_ground
        self.BUFFER_X = length
        self.BUFFER_Y = width
        super(DepthCamera, self).__init__(
            self.BUFFER_X, self.BUFFER_Y, Vec3(0.0, 0.8, 1.5), self.BKG_COLOR, pg_world.win.makeTextureBuffer,
            pg_world.makeCamera, chassis_np
        )
        self.add_to_display(pg_world, [1 / 3, 2 / 3, self.display_bottom, self.display_top])
        self.cam.lookAt(0, 2.4, 1.3)
        self.lens = self.cam.node().getLens()
        self.lens.setFov(60)
        self.lens.setAspectRatio(2.0)

        # add shader for it
        if pg_world.pg_config["headless_image"]:
            vert_path = AssetLoader.file_path(AssetLoader.asset_path, "shaders", "depth_cam_gles.vert.glsl")
            frag_path = AssetLoader.file_path(AssetLoader.asset_path, "shaders", "depth_cam_gles.frag.glsl")
        else:
            if is_mac():
                vert_path = AssetLoader.file_path(AssetLoader.asset_path, "shaders", "depth_cam_mac.vert.glsl")
                frag_path = AssetLoader.file_path(AssetLoader.asset_path, "shaders", "depth_cam_mac.frag.glsl")
            else:
                vert_path = AssetLoader.file_path(AssetLoader.asset_path, "shaders", "depth_cam.vert.glsl")
                frag_path = AssetLoader.file_path(AssetLoader.asset_path, "shaders", "depth_cam.frag.glsl")
        custom_shader = Shader.load(Shader.SL_GLSL, vertex=vert_path, fragment=frag_path)
        self.cam.node().setInitialState(RenderState.make(ShaderAttrib.make(custom_shader, 1)))

        if self.view_ground:
            self.ground = GeoMipTerrain("mySimpleTerrain")

            self.ground.setHeightfield(AssetLoader.file_path(AssetLoader.asset_path, "textures", "height_map.png"))
            # terrain.setBruteforce(True)
            # # Since the terrain is a texture, shader will not calculate the depth information, we add a moving terrain
            # # model to enable the depth information of terrain
            self.ground_model = self.ground.getRoot()
            self.ground_model.reparentTo(chassis_np)
            self.ground_model.setPos(-128, 0, 0.0)
            self.ground_model.hide(BitMask32.allOn())
            self.ground_model.show(CamMask.DepthCam)
            self.ground.generate()
