import time

from panda3d.core import NodePath, TextNode

from pgdrive.component.vehicle_module.vehicle_panel import VehiclePanel
from pgdrive.constants import RENDER_MODE_ONSCREEN, COLLISION_INFO_COLOR, COLOR, BodyName


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
        self.vehicle_panel.destroy()
        self.contact_result_render.removeNode()
        self.contact_result_render = None
        self._contact_banners = None
        self.current_banner = None
