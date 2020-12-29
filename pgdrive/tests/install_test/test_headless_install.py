from panda3d.core import loadPrcFileData

from .get_image import test_install

if __name__ == "__main__":
    loadPrcFileData("", "notify-level-task fatal")
    test_install(True)
