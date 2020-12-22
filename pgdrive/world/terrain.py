import numpy
from panda3d.bullet import BulletRigidBodyNode, BulletPlaneShape
from panda3d.core import Vec3, CardMaker, LQuaternionf, BitMask32, NodePath, TextureStage, Texture, SamplerState

from pgdrive.pg_config.body_name import BodyName
from pgdrive.pg_config.cam_mask import CamMask
from pgdrive.utils.asset_loader import AssetLoader
from pgdrive.utils.element import Element


class Terrain(Element):
    COLLISION_MASK = 2

    def __init__(self):
        super(Terrain, self).__init__()
        shape = BulletPlaneShape(Vec3(0, 0, 1), 0)
        node = BulletRigidBodyNode(BodyName.Ground)
        node.setFriction(.9)
        node.addShape(shape)

        node.setIntoCollideMask(BitMask32.bit(self.COLLISION_MASK))
        self.bullet_nodes.append(node)

        self.node_path = NodePath(node)
        if self.render:
            self.node_path.hide(CamMask.MiniMap | CamMask.Shadow | CamMask.DepthCam | CamMask.ScreenshotCam)
            # self.terrain_normal = self.loader.loadTexture(
            #     AssetLoader.file_path(AssetLoader.asset_path, "textures", "grass2", "normal.jpg")
            # )
            self.terrain_texture = self.loader.loadTexture(
                AssetLoader.file_path(AssetLoader.asset_path, "textures", "ground.png")
            )
            self.terrain_texture.setWrapU(Texture.WM_repeat)
            self.terrain_texture.setWrapV(Texture.WM_repeat)
            self.ts_color = TextureStage("color")
            self.ts_normal = TextureStage("normal")
            self.ts_normal.set_mode(TextureStage.M_normal)
            self.node_path.setPos(0, 0, 0)
            cm = CardMaker('card')
            scale = 20000
            cm.setUvRange((0, 0), (scale / 10, scale / 10))
            card = self.node_path.attachNewNode(cm.generate())
            # scale = 1 if self.use_hollow else 20000
            card.set_scale(scale)
            card.setPos(-scale / 2, -scale / 2, -0.1)
            card.setZ(-.05)
            card.setTexture(self.ts_color, self.terrain_texture)
            # card.setTexture(self.ts_normal, self.terrain_normal)
            self.terrain_texture.setMinfilter(SamplerState.FT_linear_mipmap_linear)
            self.terrain_texture.setAnisotropicDegree(8)
            card.setQuat(LQuaternionf(numpy.cos(-numpy.pi / 4), numpy.sin(-numpy.pi / 4), 0, 0))
