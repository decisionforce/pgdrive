from panda3d.core import Vec4, BitMask32

PG_EDITION = "PGDrive v0.1.4"
DEFAULT_AGENT = "default_agent"
RENDER_MODE_NONE = "none"  # Do not render
RENDER_MODE_ONSCREEN = "onscreen"  # Pop up a window and draw image in it
RENDER_MODE_OFFSCREEN = "offscreen"  # Draw image in buffer and collect image from memory

HELP_MESSAGE = "Keyboard Shortcuts:\n" \
               "  W: Acceleration\n" \
               "  S: Braking\n" \
               "  A: Moving Left\n" \
               "  D: Moving Right\n" \
               "  R: Reset the Environment\n" \
               "  H: Help Message\n" \
               "  F: Switch FPS between unlimited and realtime\n" \
               "  Esc: Quit\n"

DEBUG_MESSAGE = "  1: Box Debug Mode\n" \
                "  2: WireFrame Debug Mode\n" \
                "  3: Texture Debug Mode\n" \
                "  4: Print Node Message\n"

# priority and color
COLLISION_INFO_COLOR = dict(
    red=(0, Vec4(195 / 255, 0, 0, 1)),
    orange=(1, Vec4(218 / 255, 80 / 255, 0, 1)),
    yellow=(2, Vec4(218 / 255, 163 / 255, 0, 1)),
    green=(3, Vec4(65 / 255, 163 / 255, 0, 1))
)


class BodyName:
    Continuous_line = "Continuous Line"
    Broken_line = "Broken Line"
    Sidewalk = "Sidewalk"
    Ground = "Ground"
    Ego_vehicle = "Target Vehicle"
    Ego_vehicle_beneath = "Target Vehicle Beneath"
    Traffic_vehicle = "Traffic Vehicle"
    Lane = "Lane"
    Traffic_cone = "Traffic Cone"
    Traffic_triangle = "Traffic Triangle"


COLOR = {
    BodyName.Sidewalk: "red",
    BodyName.Continuous_line: "orange",
    BodyName.Broken_line: "yellow",
    BodyName.Traffic_vehicle: "red",
    BodyName.Traffic_cone: "orange",
    BodyName.Traffic_triangle: "orange",
    BodyName.Ego_vehicle: "red"
}


class Decoration:
    """
    Decoration lane didn't connect any nodes, they are individual or isolated.
    """
    start = "decoration"
    end = "decoration_"


class Goal:
    """
    Goal at intersection
    The keywords 0, 1, 2 should be reserved, and only be used in roundabout and intersection
    """

    RIGHT = 0
    STRAIGHT = 1
    LEFT = 2
    ADVERSE = 3  # Useless now


class CamMask:
    MainCam = BitMask32.bit(9)
    Shadow = BitMask32.bit(10)
    RgbCam = BitMask32.bit(11)
    MiniMap = BitMask32.bit(12)
    PARA_VIS = BitMask32.bit(13)
    DepthCam = BitMask32.bit(14)
    ScreenshotCam = BitMask32.bit(15)