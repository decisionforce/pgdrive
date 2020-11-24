"""

The FilterManager is a convenience class that helps with the creation
of render-to-texture buffers for image postprocessing applications.

Still need to implement:

* Make sure sort-order of buffers is correct.
* Matching buffer size to original region instead of original window.
* Intermediate layer creation.
* Handling of window clears.
* Resizing of windows.
* Do something about window-size roundoff problems.

"""
from direct.directnotify.DirectNotifyGlobal import directNotify
from direct.filter.FilterManager import FilterManager


def monkey_patch(self, win, cam, forcex=0, forcey=0):
    """ The FilterManager constructor requires you to provide
    a window which is rendering a scene, and the camera which is
    used by that window to render the scene.  These are henceforth
    called the 'original window' and the 'original camera.' """

    # Create the notify category
    # print("Monkey Patch!")
    if FilterManager.notify is None:
        FilterManager.notify = directNotify.newCategory("FilterManager")

    # Find the appropriate display region.

    region = None
    if win is not None:
        for dr in win.getDisplayRegions():
            drcam = dr.getCamera()
            if drcam == cam:
                region = dr

    # if region is None:
    #     self.notify.warning('Could not find appropriate DisplayRegion to filter')
    #     # return False
    #     return None

    # Instance Variables.

    self.win = win
    self.forcex = forcex
    self.forcey = forcey
    if win is not None:
        self.engine = win.getGsg().getEngine()
    else:
        self.engine = None
    self.region = region
    self.wclears = self.getClears(self.win)
    self.rclears = self.getClears(self.region)
    self.camera = cam
    if self.camera is not None:
        self.caminit = cam.node().getInitialState()
    else:
        self.caminit = None
    self.camstate = self.caminit
    self.buffers = []
    self.sizes = []
    if self.win is not None:
        self.nextsort = self.win.getSort() - 1000
    else:
        self.nextsort = None
    self.basex = 0
    self.basey = 0
    self.accept("window-event", self.windowEvent)


from panda3d.core import GraphicsPipe, GraphicsOutput


def getClears(self, region):
    clears = []
    if region is None:
        return clears
    for i in range(GraphicsOutput.RTPCOUNT):
        clears.append((region.getClearActive(i), region.getClearValue(i)))
    return clears


FilterManager.__init__ = monkey_patch
FilterManager.getClears = getClears
