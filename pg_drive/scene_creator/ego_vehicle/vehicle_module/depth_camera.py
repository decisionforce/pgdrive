from direct.filter.FilterManager import FilterManager, FrameBufferProperties
from panda3d.core import Vec3, NodePath, Shader, Texture
from pg_drive.pg_config.cam_mask import CamMask
from pg_drive.utils.visualization_loader import VisLoader
from pg_drive.world.image_buffer import ImageBuffer
from pg_drive.world.pg_world import PgWorld


class DepthCamera(ImageBuffer):
    # shape(dim_1, dim_2)
    BUFFER_X = 84  # dim 1
    BUFFER_Y = 84  # dim 2
    CAM_MASK = CamMask.FrontCam
    display_top = 1.0

    def __init__(self, length: int, width: int, chassis_np: NodePath, pg_world: PgWorld):
        self.BUFFER_X = length
        self.BUFFER_Y = width
        frame_buffer_props = FrameBufferProperties()
        frame_buffer_props.setFloatColor(True)
        frame_buffer_props.setRgbaBits(32, 0, 0, 0)
        super(DepthCamera, self).__init__(
            self.BUFFER_X, self.BUFFER_Y, Vec3(0.0, 0.8, 1.5), self.BKG_COLOR, pg_world.win.makeTextureBuffer,
            pg_world.makeCamera, chassis_np, frame_buffer_property=frame_buffer_props
        )
        self.add_to_display(pg_world, [0.33, 0.67, self.display_bottom, self.display_top])
        self.cam.lookAt(0, 2.4, 1.3)
        self.lens = self.cam.node().getLens()
        self.lens.setFov(60)
        # lens.setAspectRatio(2.0)

        # add shader for it
        vert_path = VisLoader.file_path(VisLoader.asset_path, "shaders", "depth_cam.vert.glsl")
        frag_path = VisLoader.file_path(VisLoader.asset_path, "shaders", "depth_cam.frag.glsl")
        custom_shader = Shader.load(Shader.SL_GLSL, vertex=vert_path, fragment=frag_path)
        # self.cam.set_shader(custom_shader)

        # # use a filter to do this
        # self.manager=FilterManager(self.buffer, self.cam)
        # self.tex_to_render = Texture()
        # quad = self.manager.renderSceneInto(colortex=self.tex_to_render)
        # quad.setShader(custom_shader)
        # quad.setShaderInput("tex", self.tex_to_render)

    def get_image(self):
        from panda3d.core import PNMImage
        img = PNMImage()
        self.tex_to_render.store(img)
        return img