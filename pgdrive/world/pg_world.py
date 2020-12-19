import logging
import os
import sys
import time
from typing import Optional, Union

import gltf
from direct.gui.OnscreenImage import OnscreenImage
from direct.showbase import ShowBase
from panda3d.bullet import BulletDebugNode, BulletWorld
from panda3d.core import Vec3, AntialiasAttrib, NodePath, loadPrcFileData, TextNode, LineSegs

from pgdrive.pg_config import PgConfig
from pgdrive.pg_config.cam_mask import CamMask
from pgdrive.utils import is_mac
from pgdrive.utils.asset_loader import AssetLoader
from pgdrive.world.constants import PG_EDITION, COLOR, COLLISION_INFO_COLOR
from pgdrive.world.force_fps import ForceFPS
from pgdrive.world.highway_render.highway_render import HighwayRender
from pgdrive.world.image_buffer import ImageBuffer
from pgdrive.world.light import Light
from pgdrive.world.onscreen_message import PgOnScreenMessage
from pgdrive.world.sky_box import SkyBox
from pgdrive.world.terrain import Terrain

root_path = os.path.dirname(os.path.dirname(__file__))


def _suppress_warning():
    loadPrcFileData("", "notify-level-glgsg fatal")
    loadPrcFileData("", "notify-level-pgraph fatal")
    loadPrcFileData("", "notify-level-pnmimage fatal")


def _free_warning():
    loadPrcFileData("", "notify-level-glgsg debug")
    loadPrcFileData("", "notify-level-pgraph debug")
    loadPrcFileData("", "notify-level-pnmimage debug")


class PgWorld(ShowBase.ShowBase):
    loadPrcFileData("", "window-title {}".format(PG_EDITION))
    loadPrcFileData("", "framebuffer-multisample 1")
    loadPrcFileData("", "multisamples 8")
    loadPrcFileData("", 'bullet-filter-algorithm groups-mask')
    loadPrcFileData("", "audio-library-name null")

    # loadPrcFileData("", " framebuffer-srgb truein")

    # loadPrcFileData("", "geom-cache-size 50000")

    # v-sync, it seems useless
    # loadPrcFileData("", "sync-video 1")

    # for debug use
    # loadPrcFileData("", "want-pstats 1")

    # loadPrcFileData("", "gl-version 3 2")

    def __init__(self, config: dict = None):
        # Setup config and Panda3d
        self.pg_config = self.default_config()
        if config is not None:
            self.pg_config.update(config)
        if self.pg_config["highway_render"]:
            # when use highway render, panda3d core will degenerate to a simple physics world
            # and the scene will be drawn by PyGame
            self.mode = "none"
            self.pg_config["use_image"] = False
        else:
            loadPrcFileData("", "win-size {} {}".format(*self.pg_config["window_size"]))
            if self.pg_config["use_render"]:
                self.mode = "onscreen"
                loadPrcFileData(
                    "", "threading-model Cull/Draw"
                )  # multi-thread render, accelerate simulation when evaluate
            else:
                self.mode = "offscreen" if self.pg_config["use_image"] else "none"
            if is_mac() and self.pg_config["use_image"]:  # Mac don't support offscreen rendering
                self.mode = "onscreen"
            if self.pg_config["headless_image"]:
                loadPrcFileData("", "load-display  pandagles2")
        _suppress_warning()

        super(PgWorld, self).__init__(windowType=self.mode)

        # Change window size at runtime if screen too small
        if self.mode != "none":
            loadPrcFileData("", "compressed-textures 1")  # Default to compress
            h = self.pipe.getDisplayHeight()
            w = self.pipe.getDisplayWidth()
            if (self.pg_config["window_size"][0] > 0.9 * w or self.pg_config["window_size"][1] > 0.9 * h):
                old_scale = self.pg_config["window_size"][0] / self.pg_config["window_size"][1]
                new_w = int(min(0.9 * w, 0.9 * h * old_scale))
                new_h = int(min(0.9 * h, 0.9 * w / old_scale))
                self.pg_config["window_size"] = tuple([new_w, new_h])
                from panda3d.core import WindowProperties
                props = WindowProperties()
                props.setSize(self.pg_config["window_size"][0], self.pg_config["window_size"][1])
                self.win.requestProperties(props)
                logging.warning(
                    "Since your screen is too small ({}, {}), we resize the window to {}.".format(
                        w, h, self.pg_config["window_size"]
                    )
                )

        # screen scale factor
        self.w_scale = max(self.pg_config["window_size"][0] / self.pg_config["window_size"][1], 1)
        self.h_scale = max(self.pg_config["window_size"][1] / self.pg_config["window_size"][0], 1)
        if self.mode == "onscreen":
            self.disableMouse()
        if not self.pg_config["highway_render"] and not self.pg_config["debug_physics_world"]:
            path = AssetLoader.windows_style2unix_style(root_path) if sys.platform == "win32" else root_path
            AssetLoader.init_loader(self, path)
            gltf.patch_loader(self.loader)
            if self.pg_config["use_render"]:
                # show logo
                self._loading_logo = OnscreenImage(
                    image=AssetLoader.file_path(AssetLoader.asset_path, "PGDrive-large.png"),
                    pos=(0, 0, 0),
                    scale=(self.w_scale, 1, self.h_scale)
                )
                self._loading_logo.setTransparency(True)
                for i in range(20):
                    self.graphicsEngine.renderFrame()
                self.taskMgr.add(self.remove_logo, "remove _loading_logo in first frame")

        self.closed = False
        self.highway_render = HighwayRender(self.pg_config["window_size"], self.pg_config["use_render"]) if \
            self.pg_config["highway_render"] else None

        # add element to render and pbr render, if is exists all the time.
        # these element will not be removed when clear_world() is called
        self.pbr_render = self.render.attachNewNode("pbrNP")

        # attach node to this root root whose children nodes will be clear after calling clear_world()
        self.worldNP = self.render.attachNewNode("world_np")

        # same as worldNP, but this node is only used for render gltf model with pbr material
        self.pbr_worldNP = self.pbr_render.attachNewNode("pbrNP")
        self.debug_node = None

        # some render attr
        self.pbrpipe = None
        self.light = None
        self.collision_info_np = None

        # physics world
        self.physics_world = BulletWorld()
        self.physics_world.setGroupCollisionFlag(0, 1, True)  # detect ego car collide terrain
        self.physics_world.setGravity(Vec3(0, 0, -9.81))  # set gravity

        # init terrain
        self.terrain = Terrain()
        self.terrain.attach_to_pg_world(self.render, self.physics_world)

        # init other world elements
        if self.mode != "none":

            # collision info render
            self.collision_info_np = NodePath(TextNode("collision_info"))
            self._init_collision_info_render()

            from pgdrive.world.our_pbr import OurPipeline
            self.pbrpipe = OurPipeline(
                render_node=None,
                window=None,
                camera_node=None,
                msaa_samples=4,
                max_lights=8,
                use_normal_maps=False,
                use_emission_maps=True,
                exposure=1.0,
                enable_shadows=False,
                enable_fog=False,
                use_occlusion_maps=False
            )
            self.pbrpipe.render_node = self.pbr_render
            self.pbrpipe.render_node.set_antialias(AntialiasAttrib.M_auto)
            self.pbrpipe._recompile_pbr()
            self.pbrpipe.manager.cleanup()

            # set main cam
            self.cam.node().setCameraMask(CamMask.MainCam)
            self.cam.node().getDisplayRegion(0).setClearColorActive(True)
            self.cam.node().getDisplayRegion(0).setClearColor(ImageBuffer.BKG_COLOR)
            lens = self.cam.node().getLens()
            lens.setFov(70)
            lens.setAspectRatio(1.2)

            self.sky_box = SkyBox()
            self.sky_box.attach_to_pg_world(self.render, self.physics_world)

            self.light = Light(self.pg_config)
            self.light.attach_to_pg_world(self.render, self.physics_world)
            self.render.setLight(self.light.direction_np)
            self.render.setLight(self.light.ambient_np)

            self.render.setShaderAuto()
            self.render.setAntialias(AntialiasAttrib.MAuto)

            # ui and render property
            if self.pg_config["show_fps"]:
                self.setFrameRateMeter(True)
            self.force_fps = ForceFPS(
                1 / (self.pg_config["physics_world_step_size"] * self.pg_config["decision_repeat"]), start=False
            )

            # onscreen message
            self.on_screen_message = PgOnScreenMessage() \
                if self.pg_config["use_render"] and self.pg_config["onscreen_message"] else None
            self._show_help_message = False
            self._episode_start_time = time.time()

            # debug setting
            self.accept('1', self.toggleDebug)
            self.accept('2', self.toggleWireframe)
            self.accept('3', self.toggleTexture)
            self.accept('4', self.toggleAnalyze)
            self.accept("h", self.toggle_help_message)
            self.accept("f", self.force_fps.toggle)

        else:
            self.on_screen_message = None

        # task manager
        self.taskMgr.remove('audioLoop')

    def _init_collision_info_render(self):
        self.collision_info_np.node().setCardActual(-5 * self.w_scale, 5.1 * self.w_scale, -0.3, 1)
        self.collision_info_np.node().setCardDecal(True)
        self.collision_info_np.node().setTextColor(1, 1, 1, 1)
        self.collision_info_np.node().setAlign(TextNode.A_center)
        self.collision_info_np.setScale(0.05)
        self.collision_info_np.setPos(-0.75 * self.w_scale, 0, -0.8 * self.h_scale)
        self.collision_info_np.reparentTo(self.aspect2d)

    def render_frame(self, text: Optional[Union[dict, str]] = None):
        """
        The real rendering is conducted by the igLoop task maintained by panda3d.
        Frame will be drawn and refresh, when taskMgr.step() is called.
        This function is only used to pass the message that needed to be printed in the screen to underlying renderer.
        :param text: A dict containing key and values or a string.
        :return: None
        """
        if not self.pg_config["highway_render"]:
            if self.on_screen_message is not None:
                self.on_screen_message.update_data(text)
            if self.pg_config["use_render"]:
                self.on_screen_message.render()
                with self.force_fps:
                    self.sky_box.step()
        else:
            return self.highway_render.draw_scene()

    def clear_world(self):
        """
        Call me to setup the whole visualization world after _init_
        """
        # attach all node to this node asset_path
        self.worldNP.node().removeAllChildren()
        self.pbr_worldNP.node().removeAllChildren()
        if self.pg_config["debug_physics_world"]:
            self.addTask(self.report_body_nums, "report_num")

        self._episode_start_time = time.time()

    @staticmethod
    def default_config():
        return PgConfig(
            dict(
                window_size=(1200, 900),  # width, height
                debug=False,
                use_render=False,
                use_image=False,
                physics_world_step_size=2e-2,
                show_fps=True,

                # show message when render is called
                onscreen_message=True,

                # limit the render fps
                # Press "f" to switch FPS, this config is deprecated!
                # force_fps=None,
                decision_repeat=5,  # This will be written by PGDriveEnv

                # only render physics world without model
                debug_physics_world=False,

                # decide the layout of white lines
                use_default_layout=True,

                # set to true only when on headless machine and use rgb image!!!!!!
                headless_image=False,

                # to shout-out to highway-env, we call the 2D-bird-view-render highway_render
                highway_render=False
            )
        )

    def step(self):
        dt = self.pg_config["physics_world_step_size"]
        self.physics_world.doPhysics(dt, 1, dt)

    def _debug_mode(self):
        debugNode = BulletDebugNode('Debug')
        debugNode.showWireframe(True)
        debugNode.showConstraints(True)
        debugNode.showBoundingBoxes(False)
        debugNode.showNormals(True)
        debugNP = self.render.attachNewNode(debugNode)
        self.physics_world.setDebugNode(debugNP.node())
        self.debug_node = debugNP

    def toggleAnalyze(self):
        self.worldNP.analyze()
        # self.worldNP.ls()

    def toggleDebug(self):
        if self.debug_node is None:
            self._debug_mode()
        if self.debug_node.isHidden():
            self.debug_node.show()
        else:
            self.debug_node.hide()

    def report_body_nums(self, task):
        logging.debug(
            "Body Nums: {}".format(
                self.physics_world.getNumRigidBodies() + self.physics_world.getNumGhosts() +
                self.physics_world.getNumVehicles()
            )
        )
        return task.done

    def close_world(self):
        self.taskMgr.stop()

        # It will report a warning said AsynTaskChain is created when taskMgr.destroy() is called but a new showbase is
        # created.
        logging.debug(
            "Before del taskMgr: task_chain_num={}, all_tasks={}".format(
                self.taskMgr.mgr.getNumTaskChains(), self.taskMgr.getAllTasks()
            )
        )
        self.taskMgr.destroy()
        logging.debug(
            "After del taskMgr: task_chain_num={}, all_tasks={}".format(
                self.taskMgr.mgr.getNumTaskChains(), self.taskMgr.getAllTasks()
            )
        )
        # while self.taskMgr.getAllTasks():
        #     time.sleep(0.1)
        self.destroy()
        self.physics_world.clearDebugNode()
        self.physics_world.clearContactAddedCallback()
        self.physics_world.clearFilterCallback()

        # del self.physics_world  # Will cause error if del it.
        self.physics_world = None

    def toggle_help_message(self):
        if self.on_screen_message:
            self.on_screen_message.toggle_help_message()

    def render_collision_info(self, contacts):
        contacts = sorted(list(contacts), key=lambda c: COLLISION_INFO_COLOR[COLOR[c]][0])
        text = contacts[0] if len(contacts) != 0 else None
        if text is None:
            text = "Normal" if time.time() - self._episode_start_time > 10 else "Press H to see help message"
            self.render_banner(text, COLLISION_INFO_COLOR["green"][1])
        else:
            self.render_banner(text, COLLISION_INFO_COLOR[COLOR[text]][1])

    def render_banner(self, text, color=COLLISION_INFO_COLOR["green"][1]):
        """
        Render the banner in the left bottom corner.
        """
        if self.collision_info_np is None:
            return
        text_node = self.collision_info_np.node()
        text_node.setCardColor(color)
        text_node.setText(text)

    def draw_line(self, start_p, end_p, color, thickness: float):
        """
        Draw line use LineSegs coordinates system. Since a resolution problem is solved, the point on screen should be
        described by [horizontal ratio, vertical ratio], each of them are ranged in [-1, 1]
        :param start_p: 2d vec
        :param end_p: 2d vec
        :param color: 4d vec, line color
        :param thickness: line thickness
        :param pg_world: pg_world class
        :return:
        """
        line_seg = LineSegs("interface")
        line_seg.setColor(*color)
        line_seg.moveTo(start_p[0] * self.w_scale, 0, start_p[1] * self.h_scale)
        line_seg.drawTo(end_p[0] * self.w_scale, 0, end_p[1] * self.h_scale)
        line_seg.setThickness(thickness)
        line_np = NodePath(line_seg.create(False)).reparentTo(self.aspect2d)
        return line_np

    def remove_logo(self, task):
        alpha = self._loading_logo.getColor()[-1]
        if alpha < 0.1:
            self._loading_logo.destroy()
            return task.done
        else:
            new_alpha = alpha - 0.08
            self._loading_logo.setColor((1, 1, 1, new_alpha))
            return task.cont


if __name__ == "__main__":
    world = PgWorld()
    world.run()
