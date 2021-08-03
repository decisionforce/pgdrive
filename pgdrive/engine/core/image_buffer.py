import logging
from typing import Union, List

import numpy as np
from panda3d.core import NodePath, Vec3, Vec4, Camera, PNMImage

from pgdrive.constants import RENDER_MODE_ONSCREEN, BKG_COLOR


class ImageBuffer:
    LINE_FRAME_COLOR = (0.8, 0.8, 0.8, 0)
    CAM_MASK = None
    BUFFER_W = 84  # left to right
    BUFFER_H = 84  # bottom to top
    BKG_COLOR = BKG_COLOR
    display_bottom = 0.8
    display_top = 1
    display_region = None
    default_region = [1 / 3, 2 / 3, 0.8, 1.0]

    def __init__(
        self,
        length: float,
        width: float,
        pos: Vec3,
        bkg_color: Union[Vec4, Vec3],
        parent_node: NodePath,
        frame_buffer_property=None,
        engine=None
    ):
        from pgdrive.engine.engine_utils import get_engine
        self.engine = engine or get_engine()
        try:
            assert self.engine.win is not None, "{} cannot be made without use_render or use_image".format(
                self.__class__.__name__
            )
            assert self.CAM_MASK is not None, "Define a camera mask for every image buffer"
        except AssertionError:
            logging.debug("Cannot create {}".format(self.__class__.__name__))
            self.buffer = None
            self.cam = NodePath(Camera("non-sense camera"))
            self.lens = self.cam.node().getLens()
            return

        if length > 100 or width > 100:
            # Too large width or length will cause corruption in Mac.
            logging.warning("You may using too large buffer! The width is {}, and length is {}.".format(width, length))

        # self.texture = Texture()
        if frame_buffer_property is None:
            self.buffer = self.engine.win.makeTextureBuffer("camera", length, width)
        else:
            self.buffer = self.engine.win.makeTextureBuffer("camera", length, width, fbp=frame_buffer_property)
            # now we have to setup a new scene graph to make this scene

        self.origin = NodePath("new render")
        self.line_borders = []
        # this takes care of setting up their camera properly
        self.cam = self.engine.makeCamera(self.buffer, clearColor=bkg_color)
        self.cam.reparentTo(self.origin)
        self.cam.setPos(pos)
        self.lens = self.cam.node().getLens()
        self.cam.node().setCameraMask(self.CAM_MASK)
        self.origin.reparentTo(parent_node)
        logging.debug("Load Image Buffer: {}".format(self.__class__.__name__))

    def get_image(self):
        """
        Bugs here! when use offscreen mode, thus the front cam obs is not from front cam now
        """
        self.engine.graphicsEngine.renderFrame()
        img = PNMImage()
        self.buffer.getScreenshot(img)
        return img

    def save_image(self, name="debug.jpg"):
        """
        for debug use
        """
        img = self.get_image()
        img.write(name)

    def get_pixels_array(self, clip=True) -> np.ndarray:
        """
        default: For gray scale image, one channel. Override this func, when you want a new obs type
        """
        img = self.get_image()

        if not clip:
            numpy_array = np.array(
                [[int(img.getGray(i, j) * 255) for j in range(img.getYSize())] for i in range(img.getXSize())],
                dtype=np.uint8
            )
            return np.clip(numpy_array, 0, 255)
        else:
            numpy_array = np.array([[img.getGray(i, j) for j in range(img.getYSize())] for i in range(img.getXSize())])
            return np.clip(numpy_array, 0, 1)

    def add_display_region(self, display_region: List[float]):
        if self.engine.global_config["use_render"]:
            # only show them when onscreen
            self.display_region = self.engine.win.makeDisplayRegion(*display_region)
            self.display_region.setCamera(self.cam)
            self.draw_border(display_region)

    def draw_border(self, display_region):
        engine = self.engine
        # add white frame for display region, convert to [-1, 1]
        left = display_region[0] * 2 - 1
        right = display_region[1] * 2 - 1
        bottom = display_region[2] * 2 - 1
        top = display_region[3] * 2 - 1

        self.line_borders.append(engine.draw_line([left, bottom], [left, top], self.LINE_FRAME_COLOR, 1.5))
        self.line_borders.append(engine.draw_line([left, top], [right, top], self.LINE_FRAME_COLOR, 1.5))
        self.line_borders.append(engine.draw_line([right, top], [right, bottom], self.LINE_FRAME_COLOR, 1.5))
        self.line_borders.append(engine.draw_line([right, bottom], [left, bottom], self.LINE_FRAME_COLOR, 1.5))

    def remove_display_region(self):
        engine = self.engine
        if engine.mode == RENDER_MODE_ONSCREEN and self.display_region:
            engine.win.removeDisplayRegion(self.display_region)
        for line_node in self.line_borders:
            line_node.detachNode()

    def destroy(self):
        engine = self.engine
        if engine is not None:
            self.remove_display_region()
            engine.graphicsEngine.removeWindow(self.buffer)
            self.display_region = None
            self.buffer = None
            if self.cam in engine.camList:
                engine.camList.remove(self.cam)
        self.cam.removeNode()
        if len(self.line_borders) != 0:
            for line_np in self.line_borders:
                if line_np:
                    line_np.removeNode()
        self.line_borders = None
        self.origin.removeNode()

    def __del__(self):
        logging.debug("{} is destroyed".format(self.__class__.__name__))
