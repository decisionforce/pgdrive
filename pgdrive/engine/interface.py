import time

from panda3d.core import NodePath, TextNode, PGTop, CardMaker, Vec3

from pgdrive.constants import RENDER_MODE_ONSCREEN, COLLISION_INFO_COLOR, COLOR, BodyName, CamMask
from pgdrive.engine.core.engine_core import EngineCore
from pgdrive.engine.core.image_buffer import ImageBuffer


class Interface:
    """
    Visualization interface, state banner and vehicle panel
    """
    def __init__(self, base_engine):
        self.engine = base_engine
        self.vehicle_panel = VehiclePanel(self.engine) if (self.engine.mode == RENDER_MODE_ONSCREEN) else None
        self.contact_result_render = self._init_collision_info_render(self.engine)
        self._contact_banners = {}  # to save time/memory
        self.current_banner = None

    def after_step(self, vehicle):
        if vehicle is not None:
            if self.vehicle_panel is not None:
                self.vehicle_panel.update_vehicle_state(vehicle)
            if self.contact_result_render is not None:
                self.render_contact_result(vehicle.contact_results)

    @staticmethod
    def _init_collision_info_render(engine):
        if engine.mode == "onscreen":
            info_np = NodePath("Collision info nodepath")
            info_np.reparentTo(engine.aspect2d)
        else:
            info_np = None
        return info_np

    def _render_banner(self, text, color=COLLISION_INFO_COLOR["green"][1]):
        """
        Render the banner in the left bottom corner.
        """
        if self.contact_result_render is None:
            return
        if self.current_banner is not None:
            self.current_banner.detachNode()
        if text in self._contact_banners:
            self._contact_banners[text].reparentTo(self.contact_result_render)
            self.current_banner = self._contact_banners[text]
        else:
            new_banner = NodePath(TextNode("collision_info:{}".format(text)))
            self._contact_banners[text] = new_banner
            text_node = new_banner.node()
            text_node.setCardColor(color)
            text_node.setText(text)
            text_node.setCardActual(-5 * self.engine.w_scale, 5.1 * self.engine.w_scale, -0.3, 1)
            text_node.setCardDecal(True)
            text_node.setTextColor(1, 1, 1, 1)
            text_node.setAlign(TextNode.A_center)
            new_banner.setScale(0.05)
            new_banner.setPos(-0.75 * self.engine.w_scale, 0, -0.8 * self.engine.h_scale)
            new_banner.reparentTo(self.contact_result_render)
            self.current_banner = new_banner

    def render_contact_result(self, contacts):
        contacts = sorted(list(contacts), key=lambda c: COLLISION_INFO_COLOR[COLOR[c]][0])
        text = contacts[0] if len(contacts) != 0 else None
        if text is None:
            text = "Normal" if time.time() - self.engine._episode_start_time > 10 else "Press H to see help message"
            self._render_banner(text, COLLISION_INFO_COLOR["green"][1])
        else:
            if text == BodyName.Base_vehicle:
                text = BodyName.Traffic_vehicle
            self._render_banner(text, COLLISION_INFO_COLOR[COLOR[text]][1])

    def remove_display_region(self):
        if self.vehicle_panel is not None:
            self.vehicle_panel.remove_display_region()
            self.vehicle_panel.buffer.set_active(False)
        if self.contact_result_render is not None:
            self.contact_result_render.detachNode()

    def add_display_region(self):
        if self.vehicle_panel is not None:
            self.vehicle_panel.add_display_region(self.vehicle_panel.default_region)
            self.vehicle_panel.buffer.set_active(True)
        if self.contact_result_render is not None:
            self.contact_result_render.reparentTo(self.engine.aspect2d)

    def destroy(self):
        self.remove_display_region()
        if self.vehicle_panel is not None:
            self.vehicle_panel.destroy()
        if self.contact_result_render is not None:
            self.contact_result_render.removeNode()
        self.contact_result_render = None
        self._contact_banners = None
        self.current_banner = None


class VehiclePanel(ImageBuffer):
    PARA_VIS_LENGTH = 12
    PARA_VIS_HEIGHT = 1
    MAX_SPEED = 120
    BUFFER_W = 2
    BUFFER_H = 1
    CAM_MASK = CamMask.PARA_VIS
    GAP = 4.1
    TASK_NAME = "update panel"
    default_region = [2 / 3, 1, ImageBuffer.display_bottom, ImageBuffer.display_top]

    def __init__(self, engine: EngineCore):
        if engine.win is None:
            return
        self.aspect2d_np = NodePath(PGTop("aspect2d"))
        self.aspect2d_np.show(self.CAM_MASK)
        self.para_vis_np = {}
        # make_buffer_func, make_camera_func = engine.win.makeTextureBuffer, engine.makeCamera

        # don't delete the space in word, it is used to set a proper position
        for i, np_name in enumerate(["Steering", " Throttle", "     Brake", "    Speed"]):
            text = TextNode(np_name)
            text.setText(np_name)
            text.setSlant(0.1)
            textNodePath = self.aspect2d_np.attachNewNode(text)
            textNodePath.setScale(0.052)
            text.setFrameColor(0, 0, 0, 1)
            text.setTextColor(0, 0, 0, 1)
            text.setFrameAsMargin(-self.GAP, self.PARA_VIS_LENGTH, 0, 0)
            text.setAlign(TextNode.ARight)
            textNodePath.setPos(-1.125111, 0, 0.9 - i * 0.08)
            if i != 0:
                cm = CardMaker(np_name)
                cm.setFrame(0, self.PARA_VIS_LENGTH - 0.21, -self.PARA_VIS_HEIGHT / 2 + 0.1, self.PARA_VIS_HEIGHT / 2)
                cm.setHasNormals(True)
                card = textNodePath.attachNewNode(cm.generate())
                card.setPos(0.21, 0, 0.22)
                self.para_vis_np[np_name.lstrip()] = card
            else:
                # left
                name = "Left"
                cm = CardMaker(name)
                cm.setFrame(
                    0, (self.PARA_VIS_LENGTH - 0.4) / 2, -self.PARA_VIS_HEIGHT / 2 + 0.1, self.PARA_VIS_HEIGHT / 2
                )
                cm.setHasNormals(True)
                card = textNodePath.attachNewNode(cm.generate())
                card.setPos(0.2 + self.PARA_VIS_LENGTH / 2, 0, 0.22)
                self.para_vis_np[name] = card
                # right
                name = "Right"
                cm = CardMaker(np_name)
                cm.setFrame(
                    -(self.PARA_VIS_LENGTH - 0.1) / 2, 0, -self.PARA_VIS_HEIGHT / 2 + 0.1, self.PARA_VIS_HEIGHT / 2
                )
                cm.setHasNormals(True)
                card = textNodePath.attachNewNode(cm.generate())
                card.setPos(0.2 + self.PARA_VIS_LENGTH / 2, 0, 0.22)
                self.para_vis_np[name] = card
        super(VehiclePanel, self).__init__(
            self.BUFFER_W,
            self.BUFFER_H,
            Vec3(-0.9, -1.01, 0.78),
            self.BKG_COLOR,
            parent_node=self.aspect2d_np,
            engine=engine
        )
        self.add_display_region(self.default_region)

    def update_vehicle_state(self, vehicle):
        steering, throttle_brake, speed = vehicle.steering, vehicle.throttle_brake, vehicle.speed
        if throttle_brake < 0:
            self.para_vis_np["Throttle"].setScale(0, 1, 1)
            self.para_vis_np["Brake"].setScale(-throttle_brake, 1, 1)
        elif throttle_brake > 0:
            self.para_vis_np["Throttle"].setScale(throttle_brake, 1, 1)
            self.para_vis_np["Brake"].setScale(0, 1, 1)
        else:
            self.para_vis_np["Throttle"].setScale(0, 1, 1)
            self.para_vis_np["Brake"].setScale(0, 1, 1)

        steering_value = abs(steering)
        if steering < 0:
            self.para_vis_np["Left"].setScale(steering_value, 1, 1)
            self.para_vis_np["Right"].setScale(0, 1, 1)
        elif steering > 0:
            self.para_vis_np["Right"].setScale(steering_value, 1, 1)
            self.para_vis_np["Left"].setScale(0, 1, 1)
        else:
            self.para_vis_np["Right"].setScale(0, 1, 1)
            self.para_vis_np["Left"].setScale(0, 1, 1)
        speed_value = speed / self.MAX_SPEED
        self.para_vis_np["Speed"].setScale(speed_value, 1, 1)

    def remove_display_region(self):
        super(VehiclePanel, self).remove_display_region()
        self.origin.detachNode()

    def add_display_region(self, display_region):
        super(VehiclePanel, self).add_display_region(display_region)
        self.origin.reparentTo(self.aspect2d_np)

    def destroy(self):
        super(VehiclePanel, self).destroy()
        for para in self.para_vis_np.values():
            para.removeNode()
        self.aspect2d_np.removeNode()